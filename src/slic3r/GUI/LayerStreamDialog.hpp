#pragma once

// LayerStream "Revise print" dialog: swaps the model of a running print from a given layer on. See docs/LayerStream.md.

#include <ctime>
#include <memory>
#include <string>

#include <wx/dialog.h>
#include <wx/timer.h>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/LayerStream/LayerStreamDiff.hpp"
#include "libslic3r/LayerStream/LayerStreamGCode.hpp"
#include "slic3r/GUI/GLModel.hpp"

class wxButton;
class wxCheckBox;
class wxTextCtrl;

namespace Slic3r {
namespace GUI {

class Plater;

// Draws the changed layer in the G-code preview: a translucent band at its height across the object and the changed regions filled.
class LayerStreamOverlay
{
public:
    static LayerStreamOverlay& get();

    void set(double z, double layer_height, const ExPolygons& changed_regions, const BoundingBoxf& footprint);
    void clear();
    bool active() const { return m_active; }
    // Called by GLCanvas3D while it renders the preview.
    void render();

private:
    bool        m_active = false;
    bool        m_dirty  = false;
    double      m_z = 0., m_height = 0.;
    ExPolygons  m_regions;
    BoundingBoxf m_footprint;
    GLModel     m_band;
    GLModel     m_fill;
    GLModel     m_outline;
};

class LayerStreamDialog : public wxDialog
{
public:
    explicit LayerStreamDialog(Plater* plater);
    ~LayerStreamDialog() override;

    // State shared with the worker thread that drives the printer.
    struct ApplyJob;

private:
    enum class Stage { Idle, WaitingForSlice, Ready, Applying, Done };

    void log(const wxString& line);
    bool pick_object();
    void on_choose_revised();
    void on_timer();
    void on_slice_finished();
    void show_result_in_preview();
    void jump_preview_to_changed_layer();
    void on_apply();
    void on_save_tail();
    void dry_check_tail();
    void update_buttons();
    LayerStream::ResumeOptions resume_options() const;

    Plater*     m_plater = nullptr;
    wxTextCtrl* m_log = nullptr;
    wxButton*   m_btn_choose = nullptr;
    wxButton*   m_btn_apply = nullptr;
    wxButton*   m_btn_save = nullptr;
    wxCheckBox* m_chk_rehome_xy = nullptr;
    wxTextCtrl* m_mesh_profile = nullptr;
    wxTimer     m_timer;

    Stage       m_stage = Stage::Idle;
    int         m_obj_idx = -1;
    int         m_vol_idx = -1;
    int         m_correction_rounds = 0;
    int         m_jump_ticks = 0;
    std::time_t m_slice_requested = 0;

    LayerStream::LayerSnapshot m_original;
    LayerStream::DiffResult    m_diff;
    LayerStream::Registration  m_registration;
    std::string                m_new_gcode_path;
    std::shared_ptr<ApplyJob>  m_job;
};

// Entry point for the menu item.
void show_layerstream_dialog(Plater* plater);

} // namespace GUI
} // namespace Slic3r
