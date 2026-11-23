#include "LayerStreamGCode.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace Slic3r {
namespace LayerStream {

namespace {

std::string trim(const std::string& s)
{
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
        ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
        --e;
    return s.substr(b, e - b);
}

std::string upper(std::string s)
{
    for (char& c : s)
        c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// The command part of a line, without the comment, trimmed.
std::string code_of(const std::string& line)
{
    const size_t semi = line.find(';');
    return trim(semi == std::string::npos ? line : line.substr(0, semi));
}

// Reads the numeric value of parameter letter p of a G-code command, e.g. "X" in "G1 X10 Y5".
bool param(const std::string& code, char p, double& out)
{
    for (size_t i = 0; i < code.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(code[i])) != p)
            continue;
        if (i > 0 && !std::isspace(static_cast<unsigned char>(code[i - 1])))
            continue;
        const char* begin = code.c_str() + i + 1;
        char*       end   = nullptr;
        const double v    = std::strtod(begin, &end);
        if (end != begin) {
            out = v;
            return true;
        }
    }
    return false;
}

std::string fmt(double v, int decimals = 3)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    std::string s(buf);
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0')
            s.pop_back();
        if (!s.empty() && s.back() == '.')
            s.pop_back();
    }
    return s.empty() ? "0" : s;
}

template<class Fn> void for_each_line(const std::string& text, size_t begin, size_t end, Fn&& fn)
{
    size_t pos = begin;
    while (pos < end) {
        size_t nl = text.find('\n', pos);
        if (nl == std::string::npos || nl > end)
            nl = end;
        std::string line = text.substr(pos, nl - pos);
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (!fn(line, pos))
            return;
        pos = nl + 1;
    }
}

} // namespace

std::vector<GCodeLayer> index_layers(const std::string& gcode)
{
    std::vector<GCodeLayer> layers;
    bool   pending = false;
    size_t pending_offset = 0;
    for_each_line(gcode, 0, gcode.size(), [&](const std::string& line, size_t offset) {
        const std::string t = trim(line);
        if (t == ";LAYER_CHANGE") {
            pending        = true;
            pending_offset = offset;
        } else if (pending && (t.rfind(";Z:", 0) == 0 || t.rfind("; Z_HEIGHT:", 0) == 0)) {
            const std::string v = t.substr(t.find(':') + 1);
            layers.push_back({std::atof(v.c_str()), pending_offset});
            pending = false;
        }
        return true;
    });
    return layers;
}

MachineState replay_state(const std::string& gcode, size_t end_offset)
{
    MachineState st;
    end_offset = std::min(end_offset, gcode.size());
    for_each_line(gcode, 0, end_offset, [&](const std::string& line, size_t) {
        const std::string code = code_of(line);
        if (code.empty())
            return true;
        const std::string u = upper(code);
        std::string cmd = u.substr(0, u.find_first_of(" \t"));
        double v = 0.;
        if (cmd == "G0" || cmd == "G1" || cmd == "G2" || cmd == "G3") {
            auto axis = [&](char a, double& dst) {
                if (param(u, a, v))
                    dst = st.absolute_xyz ? v : dst + v;
            };
            axis('X', st.x);
            axis('Y', st.y);
            axis('Z', st.z);
            if (param(u, 'E', v))
                st.e = st.absolute_e ? v : st.e + v;
            if (param(u, 'F', v))
                st.feedrate = v;
        } else if (cmd == "G90") {
            st.absolute_xyz = true;
            st.absolute_e   = true;
        } else if (cmd == "G91") {
            st.absolute_xyz = false;
            st.absolute_e   = false;
        } else if (cmd == "M82") {
            st.absolute_e = true;
        } else if (cmd == "M83") {
            st.absolute_e = false;
        } else if (cmd == "G92") {
            if (param(u, 'E', v)) st.e = v;
            if (param(u, 'X', v)) st.x = v;
            if (param(u, 'Y', v)) st.y = v;
            if (param(u, 'Z', v)) st.z = v;
        } else if (cmd == "M106") {
            if (param(u, 'S', v))
                st.fan_speed = int(std::lround(std::clamp(v, 0., 255.)));
            else
                st.fan_speed = 255;
        } else if (cmd == "M107") {
            st.fan_speed = 0;
        } else if (cmd == "M104" || cmd == "M109") {
            if (param(u, 'S', v)) st.nozzle_temp = v;
        } else if (cmd == "M140" || cmd == "M190") {
            if (param(u, 'S', v)) st.bed_temp = v;
        } else if (cmd == "EXCLUDE_OBJECT_DEFINE") {
            st.exclude_object_defines.push_back(code);
        }
        return true;
    });
    return st;
}

std::string find_forbidden_resume_command(const std::string& gcode)
{
    std::string found;
    for_each_line(gcode, 0, gcode.size(), [&](const std::string& line, size_t) {
        const std::string u = upper(code_of(line));
        if (u.empty())
            return true;
        const std::string cmd  = u.substr(0, u.find_first_of(" \t"));
        const std::string args = u.size() > cmd.size() ? u.substr(cmd.size()) : std::string();
        bool bad = cmd == "FIRMWARE_RESTART" || cmd == "RESTART" || cmd == "BED_MESH_CALIBRATE" || cmd == "G29" || cmd == "PROBE" ||
                   cmd == "START_PRINT" || cmd == "PRINT_START" || cmd == "Z_TILT_ADJUST" || cmd == "QUAD_GANTRY_LEVEL" ||
                   cmd == "SET_GCODE_OFFSET" || cmd == "G28.1" || cmd == "M502";
        if (cmd == "G28") {
            // G28 is allowed only for X and/or Y.
            const bool has_xy = args.find('X') != std::string::npos || args.find('Y') != std::string::npos;
            bad = !has_xy || args.find('Z') != std::string::npos;
        } else if (cmd == "G92") {
            // G92 E only: resetting X, Y or Z would move the coordinate frame of the part.
            bad = args.find('X') != std::string::npos || args.find('Y') != std::string::npos || args.find('Z') != std::string::npos;
        }
        if (bad) {
            found = line;
            return false;
        }
        return true;
    });
    return found;
}

const std::vector<std::string>& creality_layer_keys()
{
    static const std::vector<std::string> keys = {";LAYER:", "; layer:", "; LAYER:", ";AFTER_LAYER_CHANGE", ";LAYER_CHANGE"};
    return keys;
}

std::vector<int> firmware_layer_counter_map(const std::string& gcode, const std::vector<GCodeLayer>& layers)
{
    std::vector<int> map;
    size_t next = 0;
    int    current = -1;
    for_each_line(gcode, 0, gcode.size(), [&](const std::string& line, size_t offset) {
        while (next < layers.size() && layers[next].offset <= offset)
            current = int(next++);
        for (const std::string& key : creality_layer_keys())
            if (line.compare(0, key.size(), key) == 0) {
                // A key before the first ;LAYER_CHANGE (start G-code) counts as the first layer.
                map.push_back(std::max(current, 0));
                break;
            }
        return true;
    });
    return map;
}

int layer_for_firmware_counter(const std::vector<int>& map, int counter)
{
    if (counter < 1 || counter > int(map.size()))
        return -1;
    return map[size_t(counter - 1)];
}

int layer_with_z(const std::vector<GCodeLayer>& layers, double z)
{
    for (size_t i = 0; i < layers.size(); ++i)
        if (std::abs(layers[i].z - z) < 1e-3)
            return int(i);
    return -1;
}

TailCheck check_tail(const std::string& tail, double expected_z, size_t max_z_moves)
{
    TailCheck res;
    bool   abs_xyz = true, abs_e = true, found = false;
    double z = 0., e = 0.;
    size_t line_no = 0;
    for_each_line(tail, 0, tail.size(), [&](const std::string& line, size_t) {
        ++line_no;
        const std::string code = code_of(line);
        if (code.empty())
            return true;
        const std::string u   = upper(code);
        const std::string cmd = u.substr(0, u.find_first_of(" \t"));
        double v = 0.;
        if (cmd == "G90") abs_xyz = abs_e = true;
        else if (cmd == "G91") abs_xyz = abs_e = false;
        else if (cmd == "M82") abs_e = true;
        else if (cmd == "M83") abs_e = false;
        else if (cmd == "G92") { if (param(u, 'E', v)) e = v; }
        else if (cmd == "G0" || cmd == "G1" || cmd == "G2" || cmd == "G3") {
            if (param(u, 'Z', v)) {
                z = abs_xyz ? v : z + v;
                if (res.z_moves.size() < max_z_moves)
                    res.z_moves.push_back("line " + std::to_string(line_no) + ": " + code + "  -> Z=" + fmt(z));
            }
            const bool xy = param(u, 'X', v) || param(u, 'Y', v);
            if (param(u, 'E', v)) {
                const double de = abs_e ? v - e : v;
                e = abs_e ? v : e + v;
                if (xy && de > 0. && !found) {
                    found = true;
                    res.first_extrusion_z    = z;
                    res.first_extrusion_line = "line " + std::to_string(line_no) + ": " + code;
                }
            }
        }
        return !found || res.z_moves.size() < max_z_moves;
    });
    if (!found) {
        res.error = "The tail has no extruding move";
        return res;
    }
    if (std::abs(res.first_extrusion_z - expected_z) > 0.011) {
        res.error = "The first extrusion of the tail is at Z " + fmt(res.first_extrusion_z) + ", expected the resume layer at Z " + fmt(expected_z);
        return res;
    }
    res.ok = true;
    return res;
}

std::string resume_preamble(const MachineState& st, double layer_z, double seam_x, double seam_y, bool have_seam, const ResumeOptions& opts)
{
    const double nozzle = opts.nozzle_temp > 0. ? opts.nozzle_temp : st.nozzle_temp;
    const double bed    = opts.bed_temp > 0. ? opts.bed_temp : st.bed_temp;
    const double safe_z = layer_z + std::max(opts.z_hop_mm, 0.5);

    std::ostringstream o;
    o << "; ---- LayerStream resume preamble ----\n"
      << "; Resumes at layer Z=" << fmt(layer_z) << " with G-code sliced from a revised model, in the coordinate frame of the paused job.\n"
      << "; Does not home Z, probe the bed, change offsets or restart the firmware: there is a part on the bed.\n";
    if (bed > 0.)
        o << "M140 S" << fmt(bed, 0) << "\n";
    if (nozzle > 0.)
        o << "M104 S" << fmt(nozzle, 0) << "\n";
    if (opts.set_kinematic_position)
        o << "SET_KINEMATIC_POSITION X=" << fmt(opts.current_x) << " Y=" << fmt(opts.current_y) << " Z=" << fmt(opts.current_z)
          << " ; declare where the stopped toolhead is\n";
    o << "G90\n";
    // Absolute: from the parked height (a pause lifts the head) down to just above the part, or up if the head is lower. Both are
    // clear of the part, whose top is at most layer_z.
    o << "G1 Z" << fmt(safe_z) << " F" << fmt(opts.z_feedrate, 0) << " ; clearance above the part\n";
    if (opts.rehome_xy)
        o << "G28 X Y ; X and Y only\n";
    if (bed > 0.)
        o << "M190 S" << fmt(bed, 0) << "\n";
    if (nozzle > 0.)
        o << "M109 S" << fmt(nozzle, 0) << "\n";
    if (!opts.bed_mesh_profile.empty())
        o << "BED_MESH_PROFILE LOAD=" << opts.bed_mesh_profile << "\n";
    for (const std::string& def : st.exclude_object_defines)
        o << def << "\n";
    if (st.fan_speed > 0)
        o << "M106 S" << st.fan_speed << "\n";
    else
        o << "M107\n";
    if (have_seam)
        o << "G1 X" << fmt(seam_x) << " Y" << fmt(seam_y) << " F" << fmt(opts.travel_feedrate, 0) << " ; travel above the seam\n";
    o << "G1 Z" << fmt(layer_z) << " F" << fmt(opts.z_feedrate, 0) << " ; resume layer height\n";
    // Prime in relative mode, then restore the extrusion mode and E position the tail expects.
    o << "M83\n";
    if (opts.prime_mm > 0.)
        o << "G1 E" << fmt(opts.prime_mm) << " F300 ; prime\n";
    o << (st.absolute_e ? "M82\n" : "M83\n");
    o << "G92 E" << fmt(st.absolute_e ? st.e : 0., 5) << "\n";
    if (!st.absolute_xyz)
        o << "G91\n";
    if (st.feedrate > 0.)
        o << "G1 F" << fmt(st.feedrate, 0) << "\n";
    o << "; ---- end of LayerStream resume preamble ----\n";
    return o.str();
}

TailResult build_tail(const std::string& gcode, double swap_z, const ResumeOptions& opts)
{
    TailResult res;
    const std::vector<GCodeLayer> layers = index_layers(gcode);
    if (layers.empty()) {
        res.error = "The G-code has no ;LAYER_CHANGE markers";
        return res;
    }
    auto it = std::find_if(layers.begin(), layers.end(), [swap_z](const GCodeLayer& l) { return l.z >= swap_z - 1e-4; });
    if (it == layers.end()) {
        res.error = "The G-code has no layer at or above Z=" + fmt(swap_z);
        return res;
    }
    if (it == layers.begin()) {
        res.error = "The change starts at the first layer; nothing of the original print can be kept";
        return res;
    }
    res.layer_index = int(it - layers.begin());
    res.layer_z     = it->z;

    const MachineState st = replay_state(gcode, it->offset);

    // First XY the tail moves to, so the nozzle comes down above the seam instead of crossing the part at layer height.
    double sx = st.x, sy = st.y;
    bool   have_seam = true;
    {
        bool found = false;
        bool abs_xyz = st.absolute_xyz;
        double cx = st.x, cy = st.y;
        for_each_line(gcode, it->offset, gcode.size(), [&](const std::string& line, size_t) {
            const std::string u = upper(code_of(line));
            if (u.empty())
                return true;
            const std::string cmd = u.substr(0, u.find_first_of(" \t"));
            if (cmd == "G90") abs_xyz = true;
            if (cmd == "G91") abs_xyz = false;
            if (cmd == "G0" || cmd == "G1") {
                double v = 0.;
                bool moved = false;
                if (param(u, 'X', v)) { cx = abs_xyz ? v : cx + v; moved = true; }
                if (param(u, 'Y', v)) { cy = abs_xyz ? v : cy + v; moved = true; }
                if (moved) {
                    sx = cx; sy = cy; found = true;
                    return false;
                }
            }
            return true;
        });
        have_seam = found || (st.x != 0. || st.y != 0.);
    }

    const std::string preamble = resume_preamble(st, it->z, sx, sy, have_seam, opts);
    const std::string bad      = find_forbidden_resume_command(preamble);
    if (!bad.empty()) {
        res.error = "The resume preamble contains a forbidden command: " + bad;
        return res;
    }
    const std::string bad_tail = find_forbidden_resume_command(gcode.substr(it->offset));
    if (!bad_tail.empty()) {
        res.error = "The G-code after the resume layer contains a forbidden command: " + bad_tail;
        return res;
    }
    res.tail_bytes = gcode.size() - it->offset;
    res.gcode.reserve(preamble.size() + res.tail_bytes + 64);
    res.gcode += preamble;
    res.gcode.append(gcode, it->offset, std::string::npos);
    res.ok = true;
    return res;
}

} // namespace LayerStream
} // namespace Slic3r
