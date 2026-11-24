#include "LayerStreamDiff.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r {
namespace LayerStream {

LayerSnapshot snapshot_print(const Print& print)
{
    // Keyed by print_z rounded to 1 um, so layers of different objects at the same height merge.
    std::map<long long, ExPolygons> by_z;
    for (const PrintObject* object : print.objects()) {
        const PrintObject* source = object;
        if (source->layers().empty() && source->get_shared_object() != nullptr)
            source = source->get_shared_object();
        for (const Layer* layer : source->layers()) {
            ExPolygons& dst = by_z[std::llround(layer->print_z * 1000.)];
            for (const PrintInstance& instance : object->instances()) {
                ExPolygons islands = layer->lslices;
                Slic3r::translate(islands, instance.shift);
                append(dst, std::move(islands));
            }
        }
    }
    LayerSnapshot out;
    out.print_z.reserve(by_z.size());
    out.slices.reserve(by_z.size());
    for (auto& [z, islands] : by_z) {
        out.print_z.push_back(double(z) / 1000.);
        out.slices.push_back(union_ex(islands));
    }
    return out;
}

double area_mm2(const ExPolygons& expolys)
{
    double a = 0.;
    for (const ExPolygon& expoly : expolys)
        a += expoly.area();
    return a * SCALING_FACTOR * SCALING_FACTOR;
}

bool centroid(const ExPolygons& expolys, Vec2d& out)
{
    double sum_area = 0.;
    Vec2d  sum      = Vec2d::Zero();
    auto   add      = [&](const Polygon& poly) {
        const double a = poly.area(); // signed: holes are clockwise and subtract
        if (std::abs(a) <= 0.)
            return;
        sum += a * poly.centroid().cast<double>();
        sum_area += a;
    };
    for (const ExPolygon& expoly : expolys) {
        add(expoly.contour);
        for (const Polygon& hole : expoly.holes)
            add(hole);
    }
    if (std::abs(sum_area) <= 0.)
        return false;
    out = sum / sum_area;
    return true;
}

ExPolygons layer_difference(const ExPolygons& a, const ExPolygons& b, const DiffOptions& opts)
{
    ExPolygons sym = diff_ex(a, b);
    append(sym, diff_ex(b, a));
    if (sym.empty())
        return sym;
    const float delta = float(scale_(opts.xy_tolerance_mm));
    return delta > 0.f ? opening_ex(sym, delta) : union_ex(sym);
}

static bool layers_match(const ExPolygons& a, const ExPolygons& b, const DiffOptions& opts, ExPolygons* regions, double* area)
{
    ExPolygons d    = layer_difference(a, b, opts);
    const double ar = area_mm2(d);
    if (area != nullptr)
        *area = ar;
    if (regions != nullptr)
        *regions = std::move(d);
    return ar <= opts.min_changed_area_mm2;
}

DiffResult find_first_changed_layer(const LayerSnapshot& original, const LayerSnapshot& revised, const DiffOptions& opts)
{
    DiffResult res;
    const size_t n = std::min(original.size(), revised.size());
    for (size_t i = 0; i < n; ++i) {
        if (std::abs(original.print_z[i] - revised.print_z[i]) > opts.z_epsilon_mm) {
            res.first_changed_layer = int(i);
            res.changed_z           = revised.print_z[i];
            res.changed_regions     = layer_difference(original.slices[i], revised.slices[i], opts);
            res.changed_area_mm2    = area_mm2(res.changed_regions);
            res.reason = "Layer heights diverge at layer " + std::to_string(i + 1) + " (original Z " + std::to_string(original.print_z[i]) +
                         ", revised Z " + std::to_string(revised.print_z[i]) + ")";
            return res;
        }
        ExPolygons regions;
        double     area = 0.;
        if (!layers_match(original.slices[i], revised.slices[i], opts, &regions, &area)) {
            res.first_changed_layer = int(i);
            res.changed_z           = revised.print_z[i];
            res.changed_regions     = std::move(regions);
            res.changed_area_mm2    = area;
            res.reason = "Cross-section changes at layer " + std::to_string(i + 1) + " (" + std::to_string(area) + " mm^2 differ)";
            return res;
        }
    }
    if (original.size() == revised.size()) {
        res.identical = true;
        res.reason    = "The revised model slices identically to the original";
        return res;
    }
    // One model is taller than the other: the first extra / missing layer is the change.
    res.first_changed_layer = int(n);
    if (revised.size() > n) {
        res.changed_z       = revised.print_z[n];
        res.changed_regions = revised.slices[n];
        res.reason          = "The revised model is taller: new layers start at layer " + std::to_string(n + 1);
    } else {
        res.changed_z       = original.print_z[n];
        res.changed_regions = original.slices[n];
        res.reason          = "The revised model is shorter: it ends below layer " + std::to_string(n + 1) + " of the original";
    }
    res.changed_area_mm2 = area_mm2(res.changed_regions);
    return res;
}

void translate(LayerSnapshot& snapshot, const Vec2d& offset_mm)
{
    const Point p(coord_t(scale_(offset_mm.x())), coord_t(scale_(offset_mm.y())));
    for (ExPolygons& layer : snapshot.slices)
        Slic3r::translate(layer, p);
}

Registration register_on_plate(const LayerSnapshot& original, const LayerSnapshot& revised, const DiffOptions& opts, int max_layers)
{
    Registration reg;
    if (original.empty() || revised.empty()) {
        reg.message = "One of the prints has no layers";
        return reg;
    }
    if (std::abs(original.print_z.front() - revised.print_z.front()) > opts.z_epsilon_mm) {
        reg.message = "The first layer heights differ; slice both with the same profile";
        return reg;
    }

    // Layers whose area is unchanged are candidates for registration: a pure translation keeps the area.
    const int n = std::min<int>({max_layers, int(original.size()), int(revised.size())});
    int usable = 0;
    for (int i = 0; i < n; ++i) {
        if (std::abs(original.print_z[i] - revised.print_z[i]) > opts.z_epsilon_mm)
            break;
        const double ao = area_mm2(original.slices[i]);
        const double ar = area_mm2(revised.slices[i]);
        if (std::abs(ao - ar) > std::max(opts.min_changed_area_mm2, 1e-3 * ao))
            break;
        ++usable;
    }
    if (usable == 0) {
        reg.message = "The first layer of the revised model differs in area from the printed first layer, so it cannot be "
                      "registered on the plate. The change starts at the first layer, which is already printed.";
        return reg;
    }
    reg.layers_checked = usable;

    // Already aligned?
    bool   aligned  = true;
    double residual = 0.;
    for (int i = 0; i < usable && aligned; ++i) {
        double a = 0.;
        aligned  = layers_match(original.slices[i], revised.slices[i], opts, nullptr, &a);
        residual = std::max(residual, a);
    }
    if (aligned) {
        reg.status            = Registration::Status::Aligned;
        reg.residual_area_mm2 = residual;
        reg.message           = "The revised model sits exactly where the original is on the plate";
        return reg;
    }

    // Estimate the translation from the area weighted centroids of the first layer, then verify on every usable layer.
    Vec2d co, cr;
    if (!centroid(original.slices.front(), co) || !centroid(revised.slices.front(), cr)) {
        reg.message = "The first layer has no area";
        return reg;
    }
    const Vec2d offset_scaled = co - cr;
    const Point shift(coord_t(std::round(offset_scaled.x())), coord_t(std::round(offset_scaled.y())));
    residual = 0.;
    for (int i = 0; i < usable; ++i) {
        ExPolygons moved = revised.slices[i];
        Slic3r::translate(moved, shift);
        double a = 0.;
        if (!layers_match(original.slices[i], moved, opts, nullptr, &a)) {
            reg.residual_area_mm2 = a;
            reg.message = "The revised model's lower layers do not line up with the printed ones by moving it; it is rotated, scaled or "
                          "its base was changed. Fix the model so the printed part stays identical.";
            return reg;
        }
        residual = std::max(residual, a);
    }
    reg.status            = Registration::Status::Correctable;
    reg.offset_mm         = Vec2d(unscale<double>(shift.x()), unscale<double>(shift.y()));
    reg.residual_area_mm2 = residual;
    reg.message           = "The revised model is shifted on the plate; moving it by the computed offset aligns it with the printed part";
    return reg;
}

} // namespace LayerStream
} // namespace Slic3r
