#include "LayerStreamCreality.hpp"

#include <cstdio>
#include <stdexcept>
#include <thread>

#include <boost/asio/buffer.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http/field.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/log/trivial.hpp>
#include <nlohmann/json.hpp>

#include "AsioUtils.hpp"
#include "Http.hpp"

namespace Slic3r {
namespace LayerStream {

namespace beast     = boost::beast;
namespace http      = beast::http;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using tcp           = net::ip::tcp;
using json          = nlohmann::json;
using WsStream      = websocket::stream<beast::tcp_stream>;

namespace {

void ws_connect(net::io_context& ioc, WsStream& ws, const std::string& host_url, const std::string& port, std::chrono::milliseconds timeout)
{
    std::string host = Http::get_host_from_url(host_url);
    tcp::resolver resolver{ioc};
    beast::tcp_stream& stream = beast::get_lowest_layer(ws);
    if (const auto ec = connect_until_done(ioc, stream, resolver.resolve(host, port), timeout))
        throw beast::system_error{ec};
    host += ':' + std::to_string(stream.socket().remote_endpoint().port());
    ws.set_option(websocket::stream_base::decorator(
        [](websocket::request_type& req) { req.set(http::field::user_agent, std::string(BOOST_BEAST_VERSION_STRING) + " OrcaSlicer-LayerStream"); }));
    stream.expires_after(timeout);
    if (const auto ec = run_until_done(ioc, [&](auto&& handler) { ws.async_handshake(host, "/", handler); }))
        throw beast::system_error{ec};
    // K1-family firmware (the Ender-3 V3 KE runs it) only accepts TEXT frames.
    ws.text(true);
}

beast::error_code ws_write(net::io_context& ioc, WsStream& ws, const std::string& text)
{
    return run_until_done(ioc, [&](auto&& handler) { ws.async_write(net::buffer(text), handler); });
}

beast::error_code ws_read(net::io_context& ioc, WsStream& ws, beast::flat_buffer& buffer)
{
    return run_until_done(ioc, [&](auto&& handler) { ws.async_read(buffer, handler); });
}

void ws_close(net::io_context& ioc, WsStream& ws)
{
    beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(3));
    run_until_done(ioc, [&](auto&& handler) { ws.async_close(websocket::close_code::normal, handler); });
}

int to_int(const json& j, int def = -1)
{
    if (j.is_number())
        return j.get<int>();
    if (j.is_string()) {
        try { return std::stoi(j.get<std::string>()); } catch (...) {}
    }
    if (j.is_boolean())
        return j.get<bool>() ? 1 : 0;
    return def;
}

double to_double(const json& j, double def = 0.)
{
    if (j.is_number())
        return j.get<double>();
    if (j.is_string()) {
        try { return std::stod(j.get<std::string>()); } catch (...) {}
    }
    return def;
}

void apply_status(CrealityStatus& st, const json& msg)
{
    if (!msg.is_object())
        return;
    st.valid = true;
    if (msg.contains("state"))          st.state = to_int(msg["state"]);
    if (msg.contains("deviceState"))    st.device_state = to_int(msg["deviceState"]);
    if (msg.contains("layer"))          st.layer = to_int(msg["layer"]);
    if (msg.contains("TotalLayer"))     st.total_layers = to_int(msg["TotalLayer"]);
    if (msg.contains("printProgress"))  st.progress = to_int(msg["printProgress"]);
    if (msg.contains("printFileName") && msg["printFileName"].is_string()) st.file_name = msg["printFileName"].get<std::string>();
    if (msg.contains("model") && msg["model"].is_string())                 st.model = msg["model"].get<std::string>();
    for (const char* k : {"pause", "paused", "isPaused"})
        if (msg.contains(k))
            st.paused_flag = to_int(msg[k], 0) != 0;
    if (msg.contains("nozzleTemp"))        st.nozzle_temp = to_double(msg["nozzleTemp"]);
    if (msg.contains("targetNozzleTemp"))  st.nozzle_target = to_double(msg["targetNozzleTemp"]);
    if (msg.contains("bedTemp0"))          st.bed_temp = to_double(msg["bedTemp0"]);
    if (msg.contains("targetBedTemp0"))    st.bed_target = to_double(msg["targetBedTemp0"]);
    if (msg.contains("curPosition") && msg["curPosition"].is_string())
        st.have_position = CrealityWs::parse_position(msg["curPosition"].get<std::string>(), st.x, st.y, st.z) || st.have_position;
}

} // namespace

CrealityWs::CrealityWs(std::string host, std::string port) : m_host(std::move(host)), m_port(std::move(port)) {}

bool CrealityWs::parse_position(const std::string& s, double& x, double& y, double& z)
{
    // "X:12.00 Y:34.00 Z:5.20"
    double a = 0., b = 0., c = 0.;
    if (std::sscanf(s.c_str(), " X:%lf Y:%lf Z:%lf", &a, &b, &c) != 3)
        return false;
    x = a; y = b; z = c;
    return true;
}

CrealityStatus CrealityWs::read_status(std::chrono::milliseconds listen) const
{
    CrealityStatus  st;
    net::io_context ioc;
    WsStream        ws{ioc};
    ws_connect(ioc, ws, m_host, m_port, std::chrono::seconds(5));
    // Most firmware pushes a full status dump on connect; ask for it too, as Creality's own clients do.
    ws_write(ioc, ws, R"({"method":"get","params":{"ReqPrinterPara":1}})");
    json merged = json::object();
    const auto deadline = std::chrono::steady_clock::now() + listen;
    while (std::chrono::steady_clock::now() < deadline) {
        beast::flat_buffer buf;
        beast::get_lowest_layer(ws).expires_at(deadline);
        const beast::error_code ec = ws_read(ioc, ws, buf);
        if (ec)
            break; // timeout ends the listening window
        const std::string text = beast::buffers_to_string(buf.data());
        if (text.find("heart_beat") != std::string::npos) {
            ws_write(ioc, ws, "ok");
            continue;
        }
        const json msg = json::parse(text, nullptr, false);
        if (msg.is_object()) {
            apply_status(st, msg);
            merged.update(msg);
        }
    }
    st.raw = merged.dump();
    beast::error_code ignored;
    beast::get_lowest_layer(ws).socket().close(ignored);
    return st;
}

int CrealityWs::file_listed(const std::string& name, std::chrono::milliseconds timeout) const
{
    net::io_context ioc;
    WsStream        ws{ioc};
    ws_connect(ioc, ws, m_host, m_port, std::chrono::seconds(5));
    ws_write(ioc, ws, R"({"method":"get","params":{"reqGcodeFile":1}})");
    int        result   = -1;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        beast::flat_buffer buf;
        beast::get_lowest_layer(ws).expires_at(deadline);
        if (ws_read(ioc, ws, buf))
            break;
        const std::string text = beast::buffers_to_string(buf.data());
        if (text.find("heart_beat") != std::string::npos) {
            ws_write(ioc, ws, "ok");
            continue;
        }
        if (text.find("retGcodeFileInfo") == std::string::npos)
            continue;
        if (text.find(":" + name + ":") != std::string::npos || text.find("/" + name + "\"") != std::string::npos) {
            result = 1;
            break;
        }
        // Not (yet) listed: a printer can answer before it has indexed a file just uploaded, so ask again.
        result = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ws_write(ioc, ws, R"({"method":"get","params":{"reqGcodeFile":1}})");
    }
    beast::error_code ignored;
    beast::get_lowest_layer(ws).socket().close(ignored);
    return result;
}

void CrealityWs::send_set(const std::string& json_params) const
{
    const json params = json::parse(json_params);
    const std::string cmd = json{{"method", "set"}, {"params", params}}.dump();
    BOOST_LOG_TRIVIAL(info) << "LayerStream: -> " << cmd;
    net::io_context ioc;
    WsStream        ws{ioc};
    ws_connect(ioc, ws, m_host, m_port, std::chrono::seconds(5));
    beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(10));
    if (const auto ec = ws_write(ioc, ws, cmd))
        throw beast::system_error{ec};
    // Read one message best effort, so the close frame does not follow the command in the same burst (as CrealityPrint does).
    beast::flat_buffer buf;
    beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(2));
    ws_read(ioc, ws, buf);
    ws_close(ioc, ws);
}

void CrealityWs::pause() const { send_set(R"({"pause":1})"); }
void CrealityWs::resume() const { send_set(R"({"pause":0})"); }
void CrealityWs::stop() const { send_set(R"({"stop":1})"); }
void CrealityWs::gcode(const std::string& line) const { send_set(json{{"gcodeCmd", line}}.dump()); }

CrealityStatus CrealityWs::wait_for(const std::function<bool(const CrealityStatus&)>& pred,
                                    std::chrono::seconds                              timeout,
                                    const std::function<bool()>&                      cancelled) const
{
    const auto     deadline = std::chrono::steady_clock::now() + timeout;
    CrealityStatus st;
    while (std::chrono::steady_clock::now() < deadline) {
        if (cancelled && cancelled())
            break;
        try {
            st = read_status(std::chrono::milliseconds(1500));
            if (st.valid && pred(st))
                return st;
        } catch (const std::exception& e) {
            BOOST_LOG_TRIVIAL(warning) << "LayerStream: status read failed: " << e.what();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return st;
}

} // namespace LayerStream
} // namespace Slic3r
