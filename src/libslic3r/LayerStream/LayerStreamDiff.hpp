#pragma once

// LayerStream: mid-print model revision support.
// Compares the per-layer slices of two prints of the same plate (the job that is printing and a revised model sliced with the same
// configuration) to find the first layer that changes, and checks that the revised model sits on the plate exactly where the original
// does, so the G-code generated for the remaining layers lands on top of what was already printed.

#include <string>
#include <vector>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r {

class Print;

namespace LayerStream {

// Object islands of every layer of a print, in world (plate) coordinates, one entry per distinct print_z, sorted by print_z.
struct LayerSnapshot
{
    std::vector<double>     print_z;
    std::vector<ExPolygons> slices;

    bool   empty() const { return print_z.empty(); }
    size_t size() const { return print_z.size(); }
};

// Collects Layer::lslices of every object instance of a sliced print. Support layers are ignored: they are regenerated for the new tail.
LayerSnapshot snapshot_print(const Print& print);

struct DiffOptions
{
    // Layers whose print_z differ by less than this are the same layer.
    double z_epsilon_mm = 1e-3;
    // Differences thinner than this (mm) are slicing noise, removed by a morphological opening of the symmetric difference.
    double xy_tolerance_mm = 0.02;
    // A layer changes when the remaining symmetric difference covers more than this area (mm^2).
    double min_changed_area_mm2 = 0.05;
};

struct DiffResult
{
    bool        identical = false;      // Every layer of both snapshots matches
    int         first_changed_layer = -1; // Index into the revised snapshot (0-based), -1 when identical
    double      changed_z = 0.;           // print_z of first_changed_layer
    ExPolygons  changed_regions;          // Symmetric difference of the first changed layer, world coordinates
    double      changed_area_mm2 = 0.;
    std::string reason;                   // Human readable explanation (English, for logs)
};

// Symmetric difference of two layers with sub-tolerance noise removed.
ExPolygons layer_difference(const ExPolygons& a, const ExPolygons& b, const DiffOptions& opts);
double     area_mm2(const ExPolygons& expolys);
// Area weighted centroid in scaled coordinates. Returns false for an empty / zero area input.
bool       centroid(const ExPolygons& expolys, Vec2d& out);

DiffResult find_first_changed_layer(const LayerSnapshot& original, const LayerSnapshot& revised, const DiffOptions& opts = {});

struct Registration
{
    enum class Status {
        Aligned,       // Lower layers already coincide
        Correctable,   // Lower layers coincide after translating the revised model by offset_mm
        Blocked,       // Lower layers cannot be made to coincide by a translation
    };
    Status      status = Status::Blocked;
    Vec2d       offset_mm = Vec2d::Zero(); // Translation to apply to the revised model (world XY, mm)
    int         layers_checked = 0;
    double      residual_area_mm2 = 0.;
    std::string message;
};

// Checks that the revised model's lower layers coincide with the original's. Compares up to max_layers layers, stopping before the
// first layer that differs in a translation independent way (its area changes).
Registration register_on_plate(const LayerSnapshot& original, const LayerSnapshot& revised, const DiffOptions& opts = {}, int max_layers = 10);

// Translates every layer of a snapshot (mm).
void translate(LayerSnapshot& snapshot, const Vec2d& offset_mm);

} // namespace LayerStream
} // namespace Slic3r
