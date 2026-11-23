#pragma once

// LayerStream: builds the G-code that finishes a print from a given layer on, after the printer has printed (and stopped at) every
// layer below it. The tail is cut out of a complete G-code file sliced from the revised model and gets a resume preamble that restores
// the machine state without homing Z or probing the bed, since there is a part on the bed.

#include <string>
#include <vector>

namespace Slic3r {
namespace LayerStream {

struct GCodeLayer
{
    double z = 0.;
    size_t offset = 0; // Byte offset of the ";LAYER_CHANGE" line that starts the layer
};

// Layers of an OrcaSlicer G-code file, found by the ";LAYER_CHANGE" tag followed by ";Z:" (or "; Z_HEIGHT:").
std::vector<GCodeLayer> index_layers(const std::string& gcode);

// Machine state at a byte offset, replayed from the start of the file.
struct MachineState
{
    bool   absolute_xyz = true;
    bool   absolute_e   = true;
    double x = 0., y = 0., z = 0., e = 0.;
    double feedrate     = 0.;
    int    fan_speed    = 0;   // 0..255
    double nozzle_temp  = 0.;
    double bed_temp     = 0.;
    std::vector<std::string> exclude_object_defines; // EXCLUDE_OBJECT_DEFINE lines, replayed so EXCLUDE_OBJECT_START works
};

MachineState replay_state(const std::string& gcode, size_t end_offset);

struct ResumeOptions
{
    // Physical tool position after the interrupted job stopped, as reported by the printer. Used to declare the position to Klipper
    // with SET_KINEMATIC_POSITION when the stop disabled the steppers (Creality stock firmware does).
    bool   set_kinematic_position = true;
    double current_x = 0., current_y = 0., current_z = 0.;
    // G28 X Y: re-home X and Y only, never Z. Off by default; only use it when the toolhead may have moved while the steppers were off.
    bool   rehome_xy = false;
    // Bed mesh profile to load (Klipper BED_MESH_PROFILE LOAD=). Empty to skip.
    std::string bed_mesh_profile = "default";
    double z_hop_mm        = 2.0;   // Clearance above the printed top while travelling to the seam
    double prime_mm        = 0.3;   // Extra filament pushed after the reheat ooze; the tail's own deretraction follows
    double travel_feedrate = 9000.; // mm/min
    double z_feedrate      = 600.;  // mm/min
    // <= 0: take the temperatures that are active at the swap layer in the new G-code.
    double nozzle_temp = 0.;
    double bed_temp    = 0.;
};

struct TailResult
{
    bool        ok = false;
    std::string error;
    std::string gcode;    // Preamble + tail
    int         layer_index = -1; // 0-based index of the first layer of the tail in the source G-code
    double      layer_z = 0.;
    size_t      tail_bytes = 0;
};

// Cuts gcode from the first layer whose Z >= swap_z - 1e-4 and prepends the resume preamble.
TailResult build_tail(const std::string& gcode, double swap_z, const ResumeOptions& opts);

// The preamble alone, for a given state at the cut and the first XY the tail moves to.
std::string resume_preamble(const MachineState& state, double layer_z, double seam_x, double seam_y, bool have_seam, const ResumeOptions& opts);

// Rejects G-code that would home Z, probe the bed or restart the firmware. Returns an empty string when it is safe, else the offending line.
std::string find_forbidden_resume_command(const std::string& preamble);

} // namespace LayerStream
} // namespace Slic3r
