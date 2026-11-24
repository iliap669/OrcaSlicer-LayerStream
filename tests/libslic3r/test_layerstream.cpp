#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <string>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/LayerStream/LayerStreamDiff.hpp"
#include "libslic3r/LayerStream/LayerStreamGCode.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using namespace Slic3r::LayerStream;
using Catch::Matchers::WithinAbs;

namespace {

ExPolygons square_mm(double x, double y, double size)
{
    Polygon p{Point::new_scale(x, y), Point::new_scale(x + size, y), Point::new_scale(x + size, y + size), Point::new_scale(x, y + size)};
    return {ExPolygon(p)};
}

LayerSnapshot tower(int layers, double x, double y, double size, int widen_from = -1, double wide_size = 0.)
{
    LayerSnapshot s;
    for (int i = 0; i < layers; ++i) {
        s.print_z.push_back(0.2 * (i + 1));
        s.slices.push_back(square_mm(x, y, (widen_from >= 0 && i >= widen_from) ? wide_size : size));
    }
    return s;
}

} // namespace

TEST_CASE("LayerStream finds the first changed layer", "[LayerStream]")
{
    const LayerSnapshot original = tower(20, 100., 100., 20.);
    SECTION("identical models")
    {
        const DiffResult r = find_first_changed_layer(original, tower(20, 100., 100., 20.));
        CHECK(r.identical);
        CHECK(r.first_changed_layer == -1);
    }
    SECTION("cross-section grows from layer 12")
    {
        const DiffResult r = find_first_changed_layer(original, tower(20, 100., 100., 20., 11, 24.));
        CHECK_FALSE(r.identical);
        CHECK(r.first_changed_layer == 11);
        CHECK_THAT(r.changed_z, WithinAbs(2.4, 1e-9));
        CHECK_THAT(r.changed_area_mm2, WithinAbs(24. * 24. - 20. * 20., 0.5));
    }
    SECTION("taller revised model")
    {
        const DiffResult r = find_first_changed_layer(original, tower(25, 100., 100., 20.));
        CHECK(r.first_changed_layer == 20);
    }
    SECTION("sub-tolerance noise is ignored")
    {
        const DiffResult r = find_first_changed_layer(original, tower(20, 100.005, 100., 20.));
        CHECK(r.identical);
    }
}

TEST_CASE("LayerStream registers the revised model on the plate", "[LayerStream]")
{
    const LayerSnapshot original = tower(20, 100., 100., 20.);
    SECTION("aligned")
    {
        CHECK(register_on_plate(original, tower(20, 100., 100., 20., 11, 24.)).status == Registration::Status::Aligned);
    }
    SECTION("shifted model is correctable")
    {
        const Registration reg = register_on_plate(original, tower(20, 103., 98.5, 20., 11, 24.));
        REQUIRE(reg.status == Registration::Status::Correctable);
        CHECK_THAT(reg.offset_mm.x(), WithinAbs(-3., 1e-3));
        CHECK_THAT(reg.offset_mm.y(), WithinAbs(1.5, 1e-3));
        LayerSnapshot moved = tower(20, 103., 98.5, 20., 11, 24.);
        translate(moved, reg.offset_mm);
        CHECK(find_first_changed_layer(original, moved).first_changed_layer == 11);
    }
    SECTION("changed base is blocked")
    {
        CHECK(register_on_plate(original, tower(20, 100., 100., 20., 0, 22.)).status == Registration::Status::Blocked);
    }
}

namespace {
// Orca-style G-code for a Creality KE: 0.2 mm layers with a 0.2 mm z-hop, so Z alone cannot tell "layer 2" from "layer 1 + hop".
const std::string ke_gcode = "M140 S60\nM104 S220\nG28\nBED_MESH_CALIBRATE\nM190 S60\nM109 S220\nG90\nM82\nG92 E0\n"
                             ";LAYER_CHANGE\n;Z:0.2\n;HEIGHT:0.2\n;BEFORE_LAYER_CHANGE\n;0.2\nG92 E0\nG1 Z0.2 F600\nG1 X10 Y10 E1.5 F1200\nM106 S128\n"
                             "G1 Z0.4 ; z-hop\nG1 X20 Y20 F9000\nG1 Z0.2\nG1 X25 Y20 E2.0\n"
                             ";LAYER_CHANGE\n;Z:0.4\n;HEIGHT:0.2\n;BEFORE_LAYER_CHANGE\n;0.4\nG92 E0\nG1 Z0.4\nG1 X20 Y20 F9000\nG1 X30 Y20 E1.4\n"
                             ";LAYER_CHANGE\n;Z:0.6\n;HEIGHT:0.2\n;BEFORE_LAYER_CHANGE\n;0.6\nG92 E0\nG1 Z0.8 ; z-hop\nG1 X40 Y40\nG1 Z0.6\nG1 X50 Y40 E1.2\n"
                             "G92 E0\nG1 E-1.2 Z1.1 F1800\nG1 Z25.6 F900\nG1 X2 Y218 F3000\nM106 S0\nM104 S0\nM140 S0\nM84 X Y E\n";
} // namespace

TEST_CASE("LayerStream maps the Creality firmware layer counter to G-code layers", "[LayerStream]")
{
    const auto layers = index_layers(ke_gcode);
    REQUIRE(layers.size() == 3);
    const std::vector<int> map = firmware_layer_counter_map(ke_gcode, layers);
    // virtual_sdcard counts one line per layer (";LAYER_CHANGE"); ";BEFORE_LAYER_CHANGE" is not one of its keys.
    REQUIRE(map.size() == 3);
    CHECK(layer_for_firmware_counter(map, 1) == 0);
    CHECK(layer_for_firmware_counter(map, 2) == 1); // the failed test: firmware said layer 2 while the toolhead Z read 0.33
    CHECK(layer_for_firmware_counter(map, 3) == 2);
    CHECK(layer_for_firmware_counter(map, 0) == -1);
    CHECK(layer_for_firmware_counter(map, 4) == -1);
    CHECK(layer_with_z(layers, 0.4) == 1);
    CHECK(layer_with_z(layers, 0.5) == -1);

    // A slicer that writes both ";LAYER:n" and ";LAYER_CHANGE" makes the firmware count two per layer.
    const std::string doubled = "G28\n;LAYER_CHANGE\n;Z:0.2\n;LAYER:0\nG1 Z0.2\nG1 X1 E1\n;LAYER_CHANGE\n;Z:0.4\n;LAYER:1\nG1 Z0.4\nG1 X2 E2\n";
    const auto dl = index_layers(doubled);
    const auto dm = firmware_layer_counter_map(doubled, dl);
    REQUIRE(dm.size() == 4);
    CHECK(layer_for_firmware_counter(dm, 2) == 0);
    CHECK(layer_for_firmware_counter(dm, 3) == 1);
}

TEST_CASE("LayerStream keep-alive tail stays in the paused job's frame", "[LayerStream]")
{
    const ResumeOptions opts; // defaults: the job is paused, not stopped
    const TailResult tail = build_tail(ke_gcode, 0.4, opts);
    REQUIRE(tail.ok);
    CHECK(tail.layer_index == 1);
    CHECK_THAT(tail.layer_z, WithinAbs(0.4, 1e-6));
    // Nothing redeclares, re-homes or re-levels: the paused job's homed axes, Z offset and mesh stay valid.
    CHECK(tail.gcode.find("SET_KINEMATIC_POSITION") == std::string::npos);
    CHECK(tail.gcode.find("G28") == std::string::npos);
    CHECK(tail.gcode.find("BED_MESH") == std::string::npos);
    CHECK(tail.gcode.find("FIRMWARE_RESTART") == std::string::npos);
    CHECK(find_forbidden_resume_command(tail.gcode).empty());
    // Reheat: the stock PAUSE macro drops the nozzle to 140 C.
    CHECK(tail.gcode.find("M109 S220") != std::string::npos);
    CHECK(tail.gcode.find("G1 Z2.4 F600") != std::string::npos); // clearance first, in absolute mode
    CHECK(tail.gcode.find("G1 Z2.4") < tail.gcode.find("G1 X20 Y20"));

    // Dry check: the first extruding move of the tail is at the resume layer's Z.
    const TailCheck chk = check_tail(tail.gcode, tail.layer_z);
    REQUIRE(chk.ok);
    CHECK_THAT(chk.first_extrusion_z, WithinAbs(0.4, 1e-6));
    REQUIRE(chk.z_moves.size() >= 2);
    CHECK(chk.z_moves[0].find("Z=2.4") != std::string::npos);
    CHECK(chk.z_moves[1].find("Z=0.4") != std::string::npos);

    // The tail from the layer whose z-hop goes to 0.8 still extrudes at 0.6.
    const TailResult t3 = build_tail(ke_gcode, 0.6, opts);
    REQUIRE(t3.ok);
    const TailCheck c3 = check_tail(t3.gcode, 0.6);
    CHECK(c3.ok);
    CHECK_THAT(c3.first_extrusion_z, WithinAbs(0.6, 1e-6));
    CHECK_FALSE(check_tail(t3.gcode, 0.4).ok);
}

TEST_CASE("LayerStream tail keeps the firmware layer numbering and the slicer header", "[LayerStream]")
{
    // Second KE test: the tail started at layer 6 of 20 after a pause at firmware layer 6. SDCARD_PRINT_FILE does not reset the
    // counter, the tail's own ;LAYER_CHANGE of layer 6 was counted again, and the counter ended at 21. The tail had no header, so
    // the firmware's metadata parser could not read the layer count.
    const std::string header = "; HEADER_BLOCK_START\n; generated by OrcaSlicer 2.6.0-dev on 2026-11-24 at 10:48:51\n"
                               "; total layer number: 3\n; max_z_height: 0.60\n; HEADER_BLOCK_END\n\n; THUMBNAIL_BLOCK_START\n";
    const std::string gcode  = header + ke_gcode + "; layer_height = 0.2\n; total layers count = 3\n";
    const auto        layers = index_layers(gcode);
    REQUIRE(layers.size() == 3);
    const auto map = firmware_layer_counter_map(gcode, layers);

    ResumeOptions opts;
    opts.firmware_layer_counter = 2; // paused while printing layer 2
    const TailResult tail = build_tail(gcode, 0.4, opts);
    REQUIRE(tail.ok);
    CHECK(tail.markers_before == 1);
    CHECK(tail.disabled_markers == 1);
    CHECK(tail.counted_markers == 1);
    // The counter runs 2 (paused) -> 3 at the next layer: the revised numbering, ending at the layer count.
    CHECK(opts.firmware_layer_counter + tail.counted_markers == int(map.size()));
    CHECK(tail.gcode.rfind("; HEADER_BLOCK_START\n; generated by OrcaSlicer 2.6.0-dev on", 0) == 0);
    CHECK(tail.gcode.find("; THUMBNAIL_BLOCK_START") == std::string::npos);
    CHECK(tail.gcode.find("; total layers count = 3") != std::string::npos);
    CHECK(tail.gcode.find(";[LayerStream: counted by the firmware before the pause] ;LAYER_CHANGE\n;Z:0.4") != std::string::npos);
    CHECK(count_firmware_layer_marks(tail.gcode) == 1);
    // The dry check still sees the resume layer.
    const TailCheck chk = check_tail(tail.gcode, 0.4);
    CHECK(chk.ok);

    // Without a counter value nothing is disabled (the "Save tail" file).
    const TailResult plain = build_tail(gcode, 0.4, ResumeOptions{});
    REQUIRE(plain.ok);
    CHECK(plain.disabled_markers == 0);
    CHECK(plain.counted_markers == 2);
    // A counter below the markers before the cut disables nothing; one above the resume layer's markers disables only those.
    opts.firmware_layer_counter = 0;
    CHECK(build_tail(gcode, 0.4, opts).disabled_markers == 0);
    opts.firmware_layer_counter = 9;
    CHECK(build_tail(gcode, 0.4, opts).disabled_markers == 1);
}

TEST_CASE("LayerStream cuts a G-code tail with a resume preamble", "[LayerStream]")
{
    const std::string gcode = "M140 S60\nM104 S220\nG28\nM190 S60\nM109 S220\nG90\nM82\nG92 E0\n"
                              ";LAYER_CHANGE\n;Z:0.2\n;HEIGHT:0.2\nG1 Z0.2 F600\nG1 X10 Y10 E1.5 F1200\nM106 S128\n"
                              ";LAYER_CHANGE\n;Z:0.4\n;HEIGHT:0.2\nG1 Z0.4\nG1 X20 Y20 F9000\nG1 X30 Y20 E3.0\n"
                              ";LAYER_CHANGE\n;Z:0.6\n;HEIGHT:0.2\nG1 Z0.6\nG1 X40 Y40\nG1 X50 Y40 E4.2\n";
    REQUIRE(index_layers(gcode).size() == 3);

    ResumeOptions opts; // explicit after-stop options
    opts.set_kinematic_position = true;
    opts.current_x = 100.;
    opts.current_y = 200.;
    opts.current_z = 5.4;
    const TailResult tail = build_tail(gcode, 0.4, opts);
    REQUIRE(tail.ok);
    CHECK(tail.layer_index == 1);
    CHECK(tail.gcode.find("G92 E1.5") != std::string::npos);
    CHECK(tail.gcode.find("M106 S128") != std::string::npos);
    CHECK(tail.gcode.find("SET_KINEMATIC_POSITION X=100 Y=200 Z=5.4") != std::string::npos);
    CHECK(tail.gcode.find("\nG28\n") == std::string::npos);
    CHECK(tail.gcode.find(";Z:0.2") == std::string::npos);

    CHECK_FALSE(build_tail(gcode, 0.2, opts).ok);
    CHECK_FALSE(find_forbidden_resume_command("G28\n").empty());
    CHECK_FALSE(find_forbidden_resume_command("G28 Z\n").empty());
    CHECK(find_forbidden_resume_command("G28 X Y\n").empty());
    CHECK_FALSE(find_forbidden_resume_command("FIRMWARE_RESTART\n").empty());
    CHECK_FALSE(find_forbidden_resume_command("G92 Z0.4\n").empty());
    CHECK_FALSE(find_forbidden_resume_command("SET_GCODE_OFFSET Z=0.1\n").empty());
    CHECK(find_forbidden_resume_command("G92 E0\n").empty());
}
