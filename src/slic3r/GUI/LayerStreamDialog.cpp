#include "LayerStreamDialog.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/filedlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include <glad/gl.h>

#include "slic3r/GUI/3DScene.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GLShader.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/IMSlider.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/MsgDialog.hpp"
#include "slic3r/GUI/PartPlate.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/Utils/LayerStreamCreality.hpp"
#include "slic3r/Utils/Http.hpp"
#include "slic3r/Utils/PrintHost.hpp"

namespace fs = boost::filesystem;

namespace Slic3r {
namespace GUI {

using namespace Slic3r::LayerStream;

// ---------------------------------------------------------------------------------------------------------------------------------
// Overlay
// ---------------------------------------------------------------------------------------------------------------------------------

LayerStreamOverlay& LayerStreamOverlay::get()
{
    static LayerStreamOverlay instance;
    return instance;
}

void LayerStreamOverlay::set(double z, double layer_height, const ExPolygons& changed_regions, const BoundingBoxf& footprint)
{
    m_z         = z;
    m_height    = std::max(layer_height, 0.05);
    m_regions   = changed_regions;
    m_footprint = footprint;
    m_active    = true;
    m_dirty     = true;
}

void LayerStreamOverlay::clear()
{
    m_active = false;
    m_dirty  = true;
}

void LayerStreamOverlay::render()
{
    if (m_dirty) {
        m_band.reset();
        m_fill.reset();
        m_outline.reset();
        m_dirty = false;
        if (m_active) {
            // Band: a thin box one layer high spanning the footprint plus a margin.
            const double margin = 5.;
            const Vec2d  min    = m_footprint.defined ? m_footprint.min - Vec2d(margin, margin) : Vec2d(0., 0.);
            const Vec2d  max    = m_footprint.defined ? m_footprint.max + Vec2d(margin, margin) : Vec2d(10., 10.);
            indexed_triangle_set band = its_make_cube(max.x() - min.x(), max.y() - min.y(), m_height);
            const Vec3f shift(float(min.x()), float(min.y()), float(m_z - m_height));
            for (stl_vertex& v : band.vertices)
                v += shift;
            m_band.init_from(band);
            m_band.set_color(ColorRGBA(1.0f, 0.55f, 0.0f, 0.25f));

            if (!m_regions.empty()) {
                // Filled changed regions, slightly above the layer so they win the depth test against the toolpaths.
                const std::vector<Vec3d> tris = triangulate_expolygons_3d(m_regions, m_z + 0.02, NORMALS_UP);
                GLModel::Geometry geo;
                geo.format = {GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3};
                geo.reserve_vertices(tris.size());
                geo.reserve_indices(tris.size());
                for (size_t i = 0; i < tris.size(); ++i) {
                    geo.add_vertex(Vec3f(tris[i].cast<float>()));
                    geo.add_index((unsigned int) i);
                }
                if (!geo.is_empty()) {
                    m_fill.init_from(std::move(geo));
                    m_fill.set_color(ColorRGBA(1.0f, 0.0f, 0.0f, 0.65f));
                }
                Polygons outlines = to_polygons(m_regions);
                if (!outlines.empty()) {
                    m_outline.init_from(outlines, float(m_z + 0.03));
                    m_outline.set_color(ColorRGBA(1.0f, 0.0f, 0.0f, 1.0f));
                }
            }
        }
    }
    if (!m_active || !m_band.is_initialized())
        return;

    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader == nullptr)
        return;
    shader->start_using();
    const Camera& camera = wxGetApp().plater()->get_camera();
    shader->set_uniform("view_model_matrix", camera.get_view_matrix());
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());

    glsafe(::glEnable(GL_DEPTH_TEST));
    glsafe(::glDisable(GL_CULL_FACE));
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    glsafe(::glDepthMask(GL_FALSE));
    m_band.render();
    if (m_fill.is_initialized())
        m_fill.render();
    if (m_outline.is_initialized())
        m_outline.render();
    glsafe(::glDepthMask(GL_TRUE));
    glsafe(::glDisable(GL_BLEND));
    glsafe(::glEnable(GL_CULL_FACE));
    shader->stop_using();
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Printer job (runs on a worker thread)
// ---------------------------------------------------------------------------------------------------------------------------------

struct LayerStreamDialog::ApplyJob
{
    std::mutex               mutex;
    std::vector<std::string> lines;
    std::atomic<bool>        cancel{false};
    std::atomic<bool>        finished{false};
    std::atomic<bool>        success{false};
    std::thread              thread;

    void log(const std::string& s)
    {
        BOOST_LOG_TRIVIAL(info) << "LayerStream: " << s;
        std::lock_guard<std::mutex> lock(mutex);
        lines.push_back(s);
    }
    std::vector<std::string> take()
    {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::string> out;
        out.swap(lines);
        return out;
    }
};

namespace {

std::string fmt_mm(double v)
{
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(2);
    o << v;
    return o.str();
}

bool read_file(const std::string& path, std::string& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

bool write_file(const std::string& path, const std::string& data)
{
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f)
        return false;
    f << data;
    return bool(f);
}

std::string status_line(const CrealityStatus& st)
{
    std::string pos = st.have_position ? "X " + fmt_mm(st.x) + " Y " + fmt_mm(st.y) + " Z " + fmt_mm(st.z) : std::string("no curPosition");
    return "state " + std::to_string(st.state) + ", firmware layer " + std::to_string(st.layer) + "/" + std::to_string(st.total_layers) +
           ", " + pos + ", nozzle " + fmt_mm(st.nozzle_temp) + "/" + fmt_mm(st.nozzle_target) + ", bed " + fmt_mm(st.bed_temp) + "/" +
           fmt_mm(st.bed_target) + ", file '" + st.file_name + "'";
}

std::string file_name_of(const std::string& path)
{
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

// The whole printer side of an apply. Runs on a worker thread and talks only through job.
//
// The job is paused, never stopped: Creality's CANCEL_PRINT (what a stop runs) parks the head and disables the steppers with M84,
// after which nothing knows where the nozzle is. A paused Klipper keeps its homed axes, Z offset and bed mesh, so a tail started
// from the pause prints in exactly the frame of the original job.
void run_apply(std::shared_ptr<LayerStreamDialog::ApplyJob> job, DynamicPrintConfig printer_cfg, std::string new_gcode_path,
               std::string original_gcode_path, double changed_z, ResumeOptions opts)
{
    bool paused_by_us = false;
    auto fail = [&](const std::string& msg) {
        job->log("ERROR: " + msg);
        if (paused_by_us)
            job->log("The original job is still PAUSED and untouched (homed, heaters kept by the firmware's pause macro). "
                     "Resume it from the printer screen to finish the original model, or cancel it there.");
        job->success  = false;
        job->finished = true;
    };
    try {
        std::string gcode;
        if (!read_file(new_gcode_path, gcode))
            return fail("Cannot read the revised G-code at " + new_gcode_path);
        const std::vector<GCodeLayer> layers = index_layers(gcode);
        const int changed_idx = layer_with_z(layers, changed_z);
        if (layers.empty() || changed_idx < 0)
            return fail("The revised G-code has no layer marker at the changed Z " + fmt_mm(changed_z));

        // The firmware's "layer" value counts layer marker lines as its virtual_sdcard reads them. Count them the same way in the
        // G-code the printer is running (the copy taken when the revised model was chosen); below the change both files are equal.
        std::string running_gcode;
        const bool  have_original = !original_gcode_path.empty() && read_file(original_gcode_path, running_gcode);
        if (!have_original)
            running_gcode = gcode;
        const std::vector<GCodeLayer> run_layers  = index_layers(running_gcode);
        const std::vector<int>        counter_map = firmware_layer_counter_map(running_gcode, run_layers);
        job->log(std::string("Layer counter map built from the ") + (have_original ? "original" : "revised") + " G-code: " +
                 std::to_string(counter_map.size()) + " firmware layer marks for " + std::to_string(run_layers.size()) + " layers.");
        // Running layer index -> revised layer index, by Z.
        auto to_revised = [&](int run_idx) { return run_idx < 0 || run_idx >= int(run_layers.size()) ? -1 : layer_with_z(layers, run_layers[run_idx].z); };

        const std::string host = printer_cfg.opt_string("print_host");
        CrealityWs ws(host);

        job->log("Reading printer status from " + host + " (port 9999)...");
        CrealityStatus st = ws.read_status();
        if (!st.valid)
            return fail("The printer sent no status. Check the IP address and that the printer is on the network.");
        job->log("Before pause: " + status_line(st));
        if (!st.printing())
            return fail("The printer is not printing (state " + std::to_string(st.state) +
                        "). Resume the job first: LayerStream pauses it itself, right after a layer change.");
        if (st.layer < 1)
            return fail("The printer does not report its layer counter, so the printed layer is unknown. Nothing was sent.");
        int current = to_revised(layer_for_firmware_counter(counter_map, st.layer));
        if (current < 0)
            return fail("Firmware layer " + std::to_string(st.layer) + " does not map to a layer of this G-code. Is the printer running "
                        "the G-code of this plate? Nothing was sent.");
        if (st.have_position)
            job->log("Firmware layer " + std::to_string(st.layer) + " = G-code layer " + std::to_string(current + 1) + " at Z " +
                     fmt_mm(layers[current].z) + " (toolhead reports Z " + fmt_mm(st.z) + "; it differs during z-hops and by the Z offset).");
        if (current >= changed_idx)
            return fail("Too late: the printer is on layer " + std::to_string(current + 1) + ", the change starts at layer " +
                        std::to_string(changed_idx + 1) + ". Nothing was sent.");
        job->log("The change starts at layer " + std::to_string(changed_idx + 1) + " (Z " + fmt_mm(layers[changed_idx].z) +
                 "). Waiting for the firmware layer counter to advance, then pausing at once...");

        // Pause right after a layer change, so the restarted layer has as little as possible printed twice.
        const int start_counter = st.layer;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(30);
        CrealityStatus s;
        for (;;) {
            if (std::chrono::steady_clock::now() > deadline)
                return fail("No layer change seen within 30 minutes. Nothing was sent.");
            if (job->cancel)
                return fail("Cancelled before anything was sent to the printer");
            s = ws.read_status(std::chrono::milliseconds(700));
            if (!s.valid)
                continue;
            if (!s.printing())
                return fail("The job left the printing state (state " + std::to_string(s.state) + ") while waiting. Nothing was sent.");
            if (s.layer > start_counter)
                break;
        }
        const int next = to_revised(layer_for_firmware_counter(counter_map, s.layer));
        if (next < 0 || next > changed_idx)
            return fail("The printer moved to firmware layer " + std::to_string(s.layer) + ", past the changed layer. Nothing was sent.");
        job->log("Firmware layer " + std::to_string(s.layer) + " started (G-code layer " + std::to_string(next + 1) + "). Pausing.");
        ws.pause();
        paused_by_us = true;
        st = ws.wait_for([](const CrealityStatus& x) { return x.paused(); }, std::chrono::seconds(120));
        if (!st.paused())
            return fail("The printer did not report paused within 2 minutes. Check the printer.");
        // Let the pause macro finish its lift and park, then read the state the tail will start from.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        st = ws.read_status();
        job->log("After pause: " + status_line(st));

        // The layer the pause landed in: the firmware counter after the pause, not the toolhead Z (z-hop equals the layer height on
        // the KE profile, and the pause macro lifts Z).
        const int resume_idx = to_revised(layer_for_firmware_counter(counter_map, st.layer));
        if (resume_idx < 1)
            return fail("Cannot map the paused firmware layer " + std::to_string(st.layer) + " to a G-code layer.");
        if (resume_idx > changed_idx)
            return fail("The pause landed on layer " + std::to_string(resume_idx + 1) + ", after the changed layer " +
                        std::to_string(changed_idx + 1) + ".");
        job->log("Resume layer: " + std::to_string(resume_idx + 1) + " at Z " + fmt_mm(layers[resume_idx].z) +
                 " (it restarts from its beginning; the part of it printed before the pause is printed over).");

        opts.set_kinematic_position = false; // the frame is intact: never redeclare it
        const TailResult tail = build_tail(gcode, layers[resume_idx].z, opts);
        if (!tail.ok)
            return fail("Cannot build the tail G-code: " + tail.error);
        const TailCheck check = check_tail(tail.gcode, tail.layer_z);
        job->log("Dry check of the tail (Z moves in the paused job's frame):");
        for (const std::string& l : check.z_moves)
            job->log("  " + l);
        job->log("  first extrusion: " + check.first_extrusion_line + " at Z " + fmt_mm(check.first_extrusion_z) +
                 "; expected Z " + fmt_mm(tail.layer_z));
        if (!check.ok)
            return fail("Dry check failed: " + check.error);

        const std::string tail_name = "LayerStream_L" + std::to_string(resume_idx + 1) + "_" + std::to_string(std::time(nullptr)) + ".gcode";
        const fs::path    tail_path = fs::temp_directory_path() / tail_name;
        if (!write_file(tail_path.string(), tail.gcode))
            return fail("Cannot write " + tail_path.string());
        job->log("Tail written to " + tail_path.string() + " (" + std::to_string(tail.gcode.size() / 1024) + " KiB). Uploading (not starting)...");

        std::unique_ptr<PrintHost> printhost(PrintHost::get_print_host(&printer_cfg));
        if (!printhost)
            return fail("No print host is configured for this printer");
        PrintHostUpload upload;
        upload.source_path = tail_path;
        upload.upload_path = tail_name;
        upload.post_action = PrintHostPostUploadAction::None;
        std::string upload_error;
        const bool ok = printhost->upload(
            upload, [](Http::Progress, bool&) {}, [&](wxString err) { upload_error = into_u8(err); }, [](wxString, wxString) {});
        if (!ok)
            return fail("Upload failed: " + upload_error);
        const int listed = ws.file_listed(tail_name);
        if (listed != 1)
            return fail(listed == 0 ? "The uploaded tail is not in the printer's file list" : "Could not read the printer's file list to confirm the upload");

        // Switch the paused virtual_sdcard to the tail. SDCARD_PRINT_FILE is accepted because the pause stopped the file reader; it
        // runs only "G90" before the first line of the new file (no homing). CLEAR_PAUSE drops the paused flag of the old job.
        st = ws.read_status(std::chrono::milliseconds(1500));
        if (!st.paused())
            return fail("The printer is no longer paused (" + status_line(st) + "); not switching files.");
        job->log("Switching the paused job to " + tail_name + " (CLEAR_PAUSE, SDCARD_PRINT_FILE)...");
        paused_by_us = false; // from here the original job may already be unloaded
        ws.gcode("CLEAR_PAUSE");
        ws.gcode("SDCARD_PRINT_FILE FILENAME=" + tail_name);
        st = ws.wait_for([&](const CrealityStatus& x) { return x.printing() && x.file_name.find(tail_name) != std::string::npos; },
                         std::chrono::seconds(45));
        job->log("After switch: " + status_line(st));
        if (!(st.printing() && st.file_name.find(tail_name) != std::string::npos)) {
            job->log("ERROR: The printer did not start the tail within 45 s. Look at the printer screen: if it still shows the original "
                     "job as paused, the firmware refused the switch and you can resume or cancel that job there. If it shows an error, "
                     "cancel the job; do not start the tail from the screen by hand without checking the Z first.");
            job->success  = false;
            job->finished = true;
            return;
        }
        job->log("The printer runs the tail: it reheats, lowers to Z " + fmt_mm(tail.layer_z + std::max(opts.z_hop_mm, 0.5)) +
                 ", travels to the seam and continues at layer " + std::to_string(resume_idx + 1) + " (Z " + fmt_mm(tail.layer_z) + ").");
        job->success  = true;
        job->finished = true;
    } catch (const std::exception& e) {
        fail(e.what());
    }
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------------------
// Dialog
// ---------------------------------------------------------------------------------------------------------------------------------

LayerStreamDialog::LayerStreamDialog(Plater* plater)
    : wxDialog(wxGetApp().mainframe, wxID_ANY, _L("Revise running print (LayerStream)"), wxDefaultPosition,
               wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER)
    , m_plater(plater)
    , m_timer(this)
{
    auto* top = new wxBoxSizer(wxVERTICAL);
    auto* intro = new wxStaticText(this, wxID_ANY,
        _L("Replace the not yet printed layers of the job that is printing with layers sliced from a revised model.\n"
           "1. Keep this project, plate and settings exactly as they were sent to the printer.\n"
           "2. Choose the revised model. It replaces the object in place and the plate is sliced again.\n"
           "3. Check the changed layer marked in the preview, then apply."));
    top->Add(intro, 0, wxALL | wxEXPAND, 10);

    m_chk_rehome_xy = new wxCheckBox(this, wxID_ANY, _L("Re-home X and Y only before resuming (G28 X Y; never Z)"));
    m_chk_rehome_xy->SetValue(false);
    top->Add(m_chk_rehome_xy, 0, wxLEFT | wxRIGHT | wxBOTTOM, 10);
    auto* mesh_row = new wxBoxSizer(wxHORIZONTAL);
    mesh_row->Add(new wxStaticText(this, wxID_ANY, _L("Bed mesh profile to load (empty: keep the active mesh):")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    m_mesh_profile = new wxTextCtrl(this, wxID_ANY, "");
    mesh_row->Add(m_mesh_profile, 1);
    top->Add(mesh_row, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, 10);

    m_log = new wxTextCtrl(this, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(640), FromDIP(260)), wxTE_MULTILINE | wxTE_READONLY);
    top->Add(m_log, 1, wxLEFT | wxRIGHT | wxEXPAND, 10);

    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    m_btn_choose = new wxButton(this, wxID_ANY, _L("Choose revised model..."));
    m_btn_save   = new wxButton(this, wxID_ANY, _L("Save tail G-code..."));
    m_btn_apply  = new wxButton(this, wxID_ANY, _L("Apply changes to printer..."));
    auto* close  = new wxButton(this, wxID_CLOSE, _L("Close"));
    buttons->Add(m_btn_choose, 0, wxRIGHT, 6);
    buttons->Add(m_btn_save, 0, wxRIGHT, 6);
    buttons->Add(m_btn_apply, 0, wxRIGHT, 6);
    buttons->AddStretchSpacer();
    buttons->Add(close, 0);
    top->Add(buttons, 0, wxALL | wxEXPAND, 10);
    SetSizerAndFit(top);

    m_btn_choose->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { on_choose_revised(); });
    m_btn_apply->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { on_apply(); });
    m_btn_save->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { on_save_tail(); });
    close->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { Close(); });
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent& evt) {
        if (m_job && !m_job->finished) {
            MessageDialog dlg(this, _L("The printer is being switched to the revised G-code. Closing now cancels only the waiting step; "
                                       "a pause already sent stays in effect. Close anyway?"),
                              _L("LayerStream"), wxYES_NO | wxICON_WARNING);
            if (dlg.ShowModal() != wxID_YES) {
                evt.Veto();
                return;
            }
            m_job->cancel = true;
        }
        LayerStreamOverlay::get().clear();
        if (GLCanvas3D* canvas = m_plater->get_preview_canvas3D()) {
            canvas->set_as_dirty();
            canvas->request_extra_frame();
        }
        Destroy();
    });
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) { on_timer(); });

    if (pick_object())
        log(_L("Ready. The current plate's sliced layers are the printing job's reference."));
    update_buttons();
}

LayerStreamDialog::~LayerStreamDialog()
{
    m_timer.Stop();
    if (m_job) {
        m_job->cancel = true;
        if (m_job->thread.joinable())
            m_job->thread.detach(); // It holds its own reference to the job
    }
}

void LayerStreamDialog::log(const wxString& line)
{
    m_log->AppendText(line + "\n");
}

bool LayerStreamDialog::pick_object()
{
    Model& model = m_plater->model();
    int obj_idx = m_plater->get_selection().get_object_idx();
    if (obj_idx < 0 && model.objects.size() == 1)
        obj_idx = 0;
    if (obj_idx < 0 || obj_idx >= int(model.objects.size())) {
        log(_L("Select the object to revise (one object), then open this dialog again."));
        return false;
    }
    const ModelObject* object = model.objects[obj_idx];
    int vol_idx = -1, parts = 0;
    for (size_t i = 0; i < object->volumes.size(); ++i)
        if (object->volumes[i]->is_model_part()) {
            if (vol_idx < 0)
                vol_idx = int(i);
            ++parts;
        }
    if (parts != 1) {
        log(_L("The object must consist of exactly one part to be replaced by a revised file."));
        return false;
    }
    m_obj_idx = obj_idx;
    m_vol_idx = vol_idx;
    log(wxString::Format(_L("Object to revise: %s"), from_u8(object->name)));
    return true;
}

void LayerStreamDialog::update_buttons()
{
    const bool busy = m_stage == Stage::WaitingForSlice || m_stage == Stage::Applying;
    m_btn_choose->Enable(!busy && m_obj_idx >= 0 && m_stage != Stage::Done);
    const bool ready = m_stage == Stage::Ready && !m_diff.identical && m_diff.first_changed_layer > 0 &&
                       m_registration.status == Registration::Status::Aligned;
    m_btn_apply->Enable(ready);
    m_btn_save->Enable(ready);
}

ResumeOptions LayerStreamDialog::resume_options() const
{
    ResumeOptions opts;
    opts.set_kinematic_position = false;
    opts.rehome_xy              = m_chk_rehome_xy->GetValue();
    opts.bed_mesh_profile       = into_u8(m_mesh_profile->GetValue().Trim().Trim(false));
    return opts;
}

void LayerStreamDialog::on_choose_revised()
{
    PartPlate* plate = m_plater->get_partplate_list().get_curr_plate();
    Print*     print = plate != nullptr ? plate->fff_print() : nullptr;
    if (print == nullptr || !plate->is_slice_result_valid()) {
        log(_L("Slice the plate first: its layers are compared with the revised model."));
        return;
    }
    m_original = snapshot_print(*print);
    if (m_original.empty()) {
        log(_L("The sliced plate has no layers to compare with."));
        return;
    }
    // Keep a copy of the G-code the printer is running: its layer marks are what the firmware's layer counter counts.
    const std::string orig_gcode = plate->get_tmp_gcode_path();
    try {
        fs::remove(fs::temp_directory_path() / "LayerStream_original.gcode");
        if (fs::exists(orig_gcode))
            fs::copy_file(orig_gcode, fs::temp_directory_path() / "LayerStream_original.gcode", fs::copy_options::overwrite_existing);
    } catch (const std::exception&) {}
    log(wxString::Format(_L("Original: %d layers, top at Z %.2f mm."), int(m_original.size()), m_original.print_z.back()));

    wxFileDialog dialog(this, _L("Choose the revised model"), "", "", file_wildcards(FT_MODEL), wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK)
        return;
    const std::string path = into_u8(dialog.GetPath());
    if (!m_plater->layerstream_replace_volume(m_obj_idx, m_vol_idx, path)) {
        log(_L("Could not load the revised model (it must be a single part)."));
        return;
    }
    log(wxString::Format(_L("Loaded %s in place of the original part. Slicing..."), from_u8(path)));
    m_correction_rounds = 0;
    m_plater->select_view_3D("Preview");
    m_slice_requested = std::time(nullptr);
    m_plater->reslice();
    m_stage = Stage::WaitingForSlice;
    m_timer.Start(500);
    update_buttons();
}

void LayerStreamDialog::on_timer()
{
    if (m_stage == Stage::WaitingForSlice) {
        PartPlate* plate = m_plater->get_partplate_list().get_curr_plate();
        if (plate == nullptr || !plate->is_slice_result_valid() || m_plater->is_background_process_update_scheduled())
            return;
        const std::string path = plate->get_tmp_gcode_path();
        boost::system::error_code ec;
        if (!fs::exists(path, ec) || fs::last_write_time(path, ec) < m_slice_requested)
            return;
        m_new_gcode_path = path;
        on_slice_finished();
        return;
    }
    if (m_jump_ticks > 0) {
        if (--m_jump_ticks == 0) {
            jump_preview_to_changed_layer();
            if (m_stage == Stage::Ready)
                m_timer.Stop();
        }
    }
    if (m_stage == Stage::Applying && m_job) {
        for (const std::string& line : m_job->take())
            log(from_u8(line));
        if (m_job->finished) {
            if (m_job->thread.joinable())
                m_job->thread.join();
            m_timer.Stop();
            m_stage = m_job->success ? Stage::Done : Stage::Ready;
            log(m_job->success ? _L("Done.") : _L("Not applied."));
            update_buttons();
        }
    }
}

void LayerStreamDialog::on_slice_finished()
{
    m_timer.Stop();
    PartPlate* plate = m_plater->get_partplate_list().get_curr_plate();
    Print*     print = plate->fff_print();
    LayerSnapshot revised = snapshot_print(*print);
    log(wxString::Format(_L("Revised: %d layers, top at Z %.2f mm."), int(revised.size()), revised.empty() ? 0. : revised.print_z.back()));

    m_registration = register_on_plate(m_original, revised);
    if (m_registration.status == Registration::Status::Correctable) {
        if (m_correction_rounds >= 2) {
            m_registration.status = Registration::Status::Blocked;
            m_registration.message = "Position correction did not converge";
        } else {
            const Vec2d off = m_registration.offset_mm;
            MessageDialog dlg(this,
                              wxString::Format(_L("The revised model sits %.3f mm (X) and %.3f mm (Y) away from the printed part.\n"
                                                  "Move it so its lower layers line up exactly with what is printed?"),
                                               -off.x(), -off.y()),
                              _L("LayerStream: plate position"), wxYES_NO | wxICON_QUESTION);
            if (dlg.ShowModal() == wxID_YES) {
                ModelObject* object = m_plater->model().objects[m_obj_idx];
                for (ModelInstance* inst : object->instances)
                    inst->set_offset(inst->get_offset() + Vec3d(off.x(), off.y(), 0.));
                m_plater->changed_object(m_obj_idx);
                ++m_correction_rounds;
                log(wxString::Format(_L("Moved the revised model by %.3f, %.3f mm. Slicing again..."), off.x(), off.y()));
                m_slice_requested = std::time(nullptr);
                m_plater->reslice();
                m_stage = Stage::WaitingForSlice;
                m_timer.Start(500);
                update_buttons();
                return;
            }
            m_registration.status = Registration::Status::Blocked;
            m_registration.message = "Position correction declined";
        }
    }
    if (m_registration.status == Registration::Status::Blocked) {
        log(_L("Blocked: the revised model does not line up with the printed part.") + " " + from_u8(m_registration.message));
        m_stage = Stage::Ready;
        update_buttons();
        return;
    }
    log(wxString::Format(_L("Plate position verified on %d lower layers."), m_registration.layers_checked));

    m_diff = find_first_changed_layer(m_original, revised);
    if (m_diff.identical) {
        log(_L("The revised model slices identically to the original: nothing to apply."));
    } else if (m_diff.first_changed_layer == 0) {
        log(_L("The change starts at the first layer, which is already printed: it cannot be applied to this job."));
    } else {
        log(wxString::Format(_L("First changed layer: %d at Z %.2f mm (%.1f mm² differ). %s"), m_diff.first_changed_layer + 1,
                             m_diff.changed_z, m_diff.changed_area_mm2, from_u8(m_diff.reason)));
        show_result_in_preview();
        dry_check_tail();
    }
    m_stage = Stage::Ready;
    update_buttons();
}

void LayerStreamDialog::show_result_in_preview()
{
    BoundingBoxf footprint;
    {
        const size_t i = std::min<size_t>(m_diff.first_changed_layer, m_original.size() - 1);
        BoundingBox bb = get_extents(m_original.slices[i]);
        bb.merge(get_extents(m_diff.changed_regions));
        if (bb.defined)
            footprint = BoundingBoxf(Vec2d(unscale<double>(bb.min.x()), unscale<double>(bb.min.y())),
                                     Vec2d(unscale<double>(bb.max.x()), unscale<double>(bb.max.y())));
    }
    const double layer_h = m_diff.first_changed_layer > 0 && size_t(m_diff.first_changed_layer) < m_original.size() ?
                               m_diff.changed_z - m_original.print_z[m_diff.first_changed_layer - 1] : 0.2;
    LayerStreamOverlay::get().set(m_diff.changed_z, layer_h, m_diff.changed_regions, footprint);

    // Jump the layer slider to the changed layer once the preview has loaded the new G-code (a few timer ticks later).
    m_jump_ticks = 3;
    m_timer.Start(300);
}

void LayerStreamDialog::jump_preview_to_changed_layer()
{
    GLCanvas3D* canvas = m_plater->get_preview_canvas3D();
    if (canvas == nullptr)
        return;
    const double z      = m_diff.changed_z;
    IMSlider*    slider = canvas->get_gcode_viewer().get_layers_slider();
    if (slider != nullptr) {
        for (int i = slider->GetMinValue(); i <= slider->GetMaxValue(); ++i) {
            slider->SetHigherValue(i);
            if (slider->GetHigherValueD() >= z - 1e-4)
                break;
        }
        slider->set_as_dirty();
    }
    canvas->set_as_dirty();
    canvas->request_extra_frame();
}

void LayerStreamDialog::dry_check_tail()
{
    // Pre-apply dry check: the tail from the changed layer. On the printer the tail starts at the layer the pause lands in (at or
    // below this one), with the same preamble.
    std::string gcode;
    if (!read_file(m_new_gcode_path, gcode)) {
        log(_L("Dry check: cannot read the revised G-code."));
        return;
    }
    const TailResult tail = build_tail(gcode, m_diff.changed_z, resume_options());
    if (!tail.ok) {
        log(_L("Dry check failed:") + " " + from_u8(tail.error));
        return;
    }
    const TailCheck check = check_tail(tail.gcode, tail.layer_z);
    log(wxString::Format(_L("Dry check of a tail from layer %d (Z %.2f), first Z moves:"), tail.layer_index + 1, tail.layer_z));
    for (const std::string& l : check.z_moves)
        log("  " + from_u8(l));
    log(from_u8("  first extrusion: " + check.first_extrusion_line + " at Z " + fmt_mm(check.first_extrusion_z)));
    log(check.ok ? _L("Dry check OK: no Z homing, no position or offset change, first extrusion at the layer height.")
                 : _L("Dry check FAILED:") + " " + from_u8(check.error));
}

void LayerStreamDialog::on_save_tail()
{
    std::string gcode;
    if (!read_file(m_new_gcode_path, gcode)) {
        log(_L("Cannot read the revised G-code."));
        return;
    }
    ResumeOptions opts = resume_options();
    const TailResult tail = build_tail(gcode, m_diff.changed_z, opts);
    if (!tail.ok) {
        log(from_u8(tail.error));
        return;
    }
    wxFileDialog dlg(this, _L("Save tail G-code"), "", wxString::Format("LayerStream_from_layer_%d.gcode", tail.layer_index + 1),
                     "G-code (*.gcode)|*.gcode", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK)
        return;
    if (write_file(into_u8(dlg.GetPath()), tail.gcode))
        log(wxString::Format(_L("Saved the G-code from layer %d (Z %.2f) with its resume preamble to %s"), tail.layer_index + 1, tail.layer_z,
                             dlg.GetPath()));
    else
        log(_L("Could not write the file."));
}

void LayerStreamDialog::on_apply()
{
    DynamicPrintConfig printer_cfg = wxGetApp().preset_bundle->printers.get_edited_preset().config;
    const auto* host_type = printer_cfg.option<ConfigOptionEnum<PrintHostType>>("host_type");
    if (host_type == nullptr || host_type->value != htCrealityPrint || printer_cfg.opt_string("print_host").empty()) {
        log(_L("Set the printer's connection to Creality Print with its IP address (Printer settings > Connection) first."));
        return;
    }
    MessageDialog dlg(this,
                      wxString::Format(_L("Apply changes?\n\nThe changes start at layer %d (Z %.2f mm).\n"
                                          "LayerStream waits for the printer's next layer change and pauses the job (it is not stopped, "
                                          "so the printer keeps its position). It then uploads the remaining layers of the revised model "
                                          "to %s and switches the paused job to them. The nozzle is reheated; nothing is homed.\n"
                                          "If the printer refuses the switch, the original job stays paused and can be resumed on its screen."),
                                       m_diff.first_changed_layer + 1, m_diff.changed_z, from_u8(printer_cfg.opt_string("print_host"))),
                      _L("LayerStream"), wxYES_NO | wxICON_QUESTION);
    if (dlg.ShowModal() != wxID_YES)
        return;

    m_job = std::make_shared<ApplyJob>();
    m_job->thread = std::thread(run_apply, m_job, printer_cfg, m_new_gcode_path,
                                (fs::temp_directory_path() / "LayerStream_original.gcode").string(), m_diff.changed_z, resume_options());
    m_stage = Stage::Applying;
    m_timer.Start(300);
    update_buttons();
}

void show_layerstream_dialog(Plater* plater)
{
    if (plater == nullptr)
        return;
    auto* dlg = new LayerStreamDialog(plater);
    dlg->Show();
}

} // namespace GUI
} // namespace Slic3r
