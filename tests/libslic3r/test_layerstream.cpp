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

TEST_CASE("LayerStream cuts a G-code tail with a resume preamble", "[LayerStream]")
{
    const std::string gcode = "M140 S60\nM104 S220\nG28\nM190 S60\nM109 S220\nG90\nM82\nG92 E0\n"
                              ";LAYER_CHANGE\n;Z:0.2\n;HEIGHT:0.2\nG1 Z0.2 F600\nG1 X10 Y10 E1.5 F1200\nM106 S128\n"
                              ";LAYER_CHANGE\n;Z:0.4\n;HEIGHT:0.2\nG1 Z0.4\nG1 X20 Y20 F9000\nG1 X30 Y20 E3.0\n"
                              ";LAYER_CHANGE\n;Z:0.6\n;HEIGHT:0.2\nG1 Z0.6\nG1 X40 Y40\nG1 X50 Y40 E4.2\n";
    REQUIRE(index_layers(gcode).size() == 3);

    ResumeOptions opts;
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
}
