#pragma once

// LayerStream: talks to a Creality printer on stock (Creality OS / Klipper based) firmware, such as the Ender-3 V3 KE, K1 and K2
// families, over the same port 9999 WebSocket that OrcaSlicer's CrealityPrint host uses to start prints.
//
// The stock firmware exposes no Moonraker API and no way to feed G-code into a running job: a job is a file that the printer reads
// from its own storage. So a mid-print revision is applied as: pause the job, stop it, upload a "tail" file that holds the G-code
// of the remaining layers behind a resume preamble, and start that file. See docs/LayerStream.md.

#include <chrono>
#include <functional>
#include <string>

namespace Slic3r {
namespace LayerStream {

struct CrealityStatus
{
    bool        valid = false;
    int         state = -1;          // 0 idle/processing, 1 printing, 2 finished, 4 stopped, 5 paused (ha_creality_ws findings)
    int         device_state = -1;   // 7 homing
    int         layer = -1;
    int         total_layers = -1;
    int         progress = -1;
    bool        paused_flag = false;
    bool        have_position = false;
    double      x = 0., y = 0., z = 0.;
    double      nozzle_temp = 0., nozzle_target = 0., bed_temp = 0., bed_target = 0.;
    std::string file_name;
    std::string model;
    std::string raw;                 // Merged raw status JSON, for logs

    bool printing() const { return state == 1; }
    bool paused() const { return state == 5 || paused_flag; }
    bool busy() const { return printing() || paused() || state == 7 || state == 9; }
};

class CrealityWs
{
public:
    // host: the printer's address as configured in OrcaSlicer ("192.168.1.20" or "http://192.168.1.20").
    explicit CrealityWs(std::string host, std::string port = "9999");

    // Collects the status the printer pushes for up to `listen` (it sends a full dump on connect, then deltas).
    CrealityStatus read_status(std::chrono::milliseconds listen = std::chrono::milliseconds(2500)) const;

    // {"method":"set","params":{"pause":1}} / {"pause":0} / {"stop":1}
    void pause() const;
    void resume() const;
    void stop() const;
    // {"method":"set","params":{"gcodeCmd":"..."}}: one console G-code line (Klipper syntax on stock firmware).
    void gcode(const std::string& line) const;

    // Polls read_status() until pred is true or the timeout passes. Returns the last status.
    CrealityStatus wait_for(const std::function<bool(const CrealityStatus&)>& pred,
                            std::chrono::seconds                              timeout,
                            const std::function<bool()>&                      cancelled = {}) const;

    static bool parse_position(const std::string& cur_position, double& x, double& y, double& z);

private:
    void send_set(const std::string& json_params) const;

    std::string m_host;
    std::string m_port;
};

} // namespace LayerStream
} // namespace Slic3r
