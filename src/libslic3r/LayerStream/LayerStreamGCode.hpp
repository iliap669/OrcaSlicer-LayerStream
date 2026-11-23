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
    // Default mode ("keep alive"): the interrupted job is only PAUSED, so Klipper keeps its homed axes, G-code offset (Z offset) and
    // bed mesh, and the tail is started from the paused state. Positions in the tail are then in exactly the same frame as the
    // original job, and nothing may redeclare or re-home the position.
    //
    // The options below only make sense after a STOP, which on Creality stock firmware runs CANCEL_PRINT -> M84 and loses the
    // position. They are kept for experiments and are off by default; the GUI does not use them.
    bool   set_kinematic_position = false;
    double current_x = 0., current_y = 0., current_z = 0.;
    bool   rehome_xy = false; // G28 X Y only, never Z
    // Bed mesh profile to load (Klipper BED_MESH_PROFILE LOAD=). Empty keeps the mesh that is active, which is right after a pause:
    // loading "default" could replace the mesh the start G-code just probed with an older saved one.
    std::string bed_mesh_profile;
    double z_hop_mm        = 2.0;   // Clearance above the resume layer while travelling to the seam
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

// Rejects G-code that would home Z, probe the bed, restart the firmware or shift the coordinate frame (G92 X/Y/Z, SET_GCODE_OFFSET).
// Returns an empty string when it is safe, else the offending line. G28 X/Y is accepted (the preamble only emits it when asked).
std::string find_forbidden_resume_command(const std::string& gcode);

// Creality's virtual_sdcard (klippy/extras/virtual_sdcard.py on K1/KE stock firmware) counts a layer for every line that starts with
// one of these, and reports the count as the "layer" status value.
const std::vector<std::string>& creality_layer_keys();

// For every value of the firmware layer counter (index = counter - 1) the 0-based index into `layers` of the layer that is being
// printed when the counter has that value. Built by counting the same lines as the firmware.
std::vector<int> firmware_layer_counter_map(const std::string& gcode, const std::vector<GCodeLayer>& layers);
// -1 when the counter is outside the map.
int layer_for_firmware_counter(const std::vector<int>& map, int counter);

// Index of the layer whose Z equals z (within 1e-3), or -1.
int layer_with_z(const std::vector<GCodeLayer>& layers, double z);

// Dry check of a tail: the first lines that move Z, and the Z of the first extruding XY move, which must be the resume layer's Z.
struct TailCheck
{
    bool                     ok = false;
    std::string              error;
    double                   first_extrusion_z = 0.;
    std::string              first_extrusion_line;
    std::vector<std::string> z_moves; // "line N: <code>  -> Z=<z>"
};
TailCheck check_tail(const std::string& tail, double expected_z, size_t max_z_moves = 12);

} // namespace LayerStream
} // namespace Slic3r
