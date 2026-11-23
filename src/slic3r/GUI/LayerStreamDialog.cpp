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

// Index of the G-code layer being printed at height z (the highest layer at or below z, tolerating a Z hop above it).
int layer_at(const std::vector<GCodeLayer>& layers, double z)
{
    int idx = -1;
    for (size_t i = 0; i < layers.size(); ++i)
        if (layers[i].z <= z + 0.01)
            idx = int(i);
    return idx;
}

// The whole printer side of an apply. Runs on a worker thread and talks only through job.
void run_apply(std::shared_ptr<LayerStreamDialog::ApplyJob> job, DynamicPrintConfig printer_cfg, std::string new_gcode_path,
               double changed_z, ResumeOptions opts)
{
    auto fail = [&](const std::string& msg) {
        job->log("ERROR: " + msg);
        job->success = false;
        job->finished = true;
    };
    try {
        std::string gcode;
        if (!read_file(new_gcode_path, gcode))
            return fail("Cannot read the revised G-code at " + new_gcode_path);
        const std::vector<GCodeLayer> layers = index_layers(gcode);
        const int changed_idx = layer_at(layers, changed_z);
        if (layers.empty() || changed_idx < 0)
            return fail("The revised G-code has no layer markers");

        const std::string host = printer_cfg.opt_string("print_host");
        CrealityWs ws(host);

        job->log("Reading printer status from " + host + " (port 9999)...");
        CrealityStatus st = ws.read_status();
        if (!st.valid)
            return fail("The printer sent no status. Check the IP address and that the printer is on the network.");
        job->log("Printer state " + std::to_string(st.state) + ", file '" + st.file_name + "', Z " + fmt_mm(st.z) + ", layer " +
                 std::to_string(st.layer) + "/" + std::to_string(st.total_layers));
        if (!st.printing())
            return fail("The printer is not printing (state " + std::to_string(st.state) +
                        "). Resume the job first: LayerStream pauses it itself, right after a layer change.");
        if (!st.have_position)
            return fail("The printer does not report its toolhead position (curPosition), so the printed height is unknown.");

        // Wait for the next layer change, so the pause lands early in a layer and the tail restarts that layer from its beginning.
        int current = layer_at(layers, st.z);
        if (current >= changed_idx)
            return fail("Too late: the printer is at layer " + std::to_string(current + 1) + " (Z " + fmt_mm(st.z) +
                        "), the change starts at layer " + std::to_string(changed_idx + 1) + ", which is already printed or printing.");
        job->log("Printing layer " + std::to_string(current + 1) + "; the change starts at layer " + std::to_string(changed_idx + 1) +
                 ". Waiting for the next layer change to pause...");
        int  resume_idx = -1;
        int  candidate  = -1;
        auto deadline   = std::chrono::steady_clock::now() + std::chrono::minutes(30);
        while (std::chrono::steady_clock::now() < deadline) {
            if (job->cancel)
                return fail("Cancelled before anything was sent to the printer");
            CrealityStatus s = ws.read_status(std::chrono::milliseconds(800));
            if (!s.valid || !s.have_position)
                continue;
            if (!s.printing())
                return fail("The job left the printing state (state " + std::to_string(s.state) + ") while waiting");
            const int l = layer_at(layers, s.z);
            if (l > current) {
                // Seen twice in a row, so a Z hop during a travel is not taken for a layer change.
                if (l == candidate) {
                    resume_idx = l;
                    break;
                }
                candidate = l;
            } else
                candidate = -1;
        }
        if (resume_idx < 0)
            return fail("No layer change seen within 30 minutes");
        if (resume_idx > changed_idx)
            return fail("Too late: the printer moved past the changed layer while waiting");

        job->log("Layer " + std::to_string(resume_idx + 1) + " started at Z " + fmt_mm(layers[resume_idx].z) + ". Pausing.");
        ws.pause();
        st = ws.wait_for([](const CrealityStatus& s) { return s.paused(); }, std::chrono::seconds(120));
        if (!st.paused())
            return fail("The printer did not report paused within 2 minutes. Nothing else was sent; check the printer.");
        job->log("Paused. Stopping the original job (the stock firmware cannot switch files while a job is loaded)...");

        ws.stop();
        st = ws.wait_for([](const CrealityStatus& s) { return !s.busy() && s.have_position; }, std::chrono::seconds(180));
        if (st.busy())
            return fail("The printer did not finish stopping within 3 minutes. Check it before continuing.");
        job->log("Stopped. Toolhead at X " + fmt_mm(st.x) + " Y " + fmt_mm(st.y) + " Z " + fmt_mm(st.z));
        if (st.z < layers[resume_idx].z) {
            return fail("The reported Z after the stop (" + fmt_mm(st.z) + ") is below the resume layer (" + fmt_mm(layers[resume_idx].z) +
                        "). Not starting the tail, as the nozzle could hit the part.");
        }

        opts.current_x = st.x;
        opts.current_y = st.y;
        opts.current_z = st.z;
        const TailResult tail = build_tail(gcode, layers[resume_idx].z, opts);
        if (!tail.ok)
            return fail("Cannot build the tail G-code: " + tail.error);

        const fs::path tail_path = fs::temp_directory_path() /
                                   ("LayerStream_L" + std::to_string(resume_idx + 1) + "_" + std::to_string(std::time(nullptr)) + ".gcode");
        if (!write_file(tail_path.string(), tail.gcode))
            return fail("Cannot write " + tail_path.string());
        job->log("Tail G-code from layer " + std::to_string(resume_idx + 1) + " written to " + tail_path.string() + " (" +
                 std::to_string(tail.gcode.size() / 1024) + " KiB). Uploading and starting it...");

        std::unique_ptr<PrintHost> printhost(PrintHost::get_print_host(&printer_cfg));
        if (!printhost)
            return fail("No print host is configured for this printer");
        PrintHostUpload upload;
        upload.source_path = tail_path;
        upload.upload_path = tail_path.filename();
        upload.post_action = PrintHostPostUploadAction::StartPrint;
        std::string upload_error;
        const bool ok = printhost->upload(
            upload, [](Http::Progress, bool&) {}, [&](wxString err) { upload_error = into_u8(err); }, [](wxString, wxString) {});
        if (!ok)
            return fail("Upload / start failed: " + upload_error + ". The tail file is kept at " + tail_path.string() +
                        "; you can start it from the printer screen.");
        job->log("The printer is reheating and continues from layer " + std::to_string(resume_idx + 1) + " with the revised model.");
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

    m_chk_kinematic = new wxCheckBox(this, wxID_ANY, _L("Declare the stopped toolhead position (SET_KINEMATIC_POSITION, needs force_move)"));
    m_chk_kinematic->SetValue(true);
    m_chk_rehome_xy = new wxCheckBox(this, wxID_ANY, _L("Re-home X and Y only before resuming (G28 X Y; never Z)"));
    m_chk_rehome_xy->SetValue(false);
    top->Add(m_chk_kinematic, 0, wxLEFT | wxRIGHT | wxBOTTOM, 10);
    top->Add(m_chk_rehome_xy, 0, wxLEFT | wxRIGHT | wxBOTTOM, 10);
    auto* mesh_row = new wxBoxSizer(wxHORIZONTAL);
    mesh_row->Add(new wxStaticText(this, wxID_ANY, _L("Bed mesh profile to load:")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    m_mesh_profile = new wxTextCtrl(this, wxID_ANY, "default");
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
                                       "a pause or stop already sent stays in effect. Close anyway?"),
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
    opts.set_kinematic_position = m_chk_kinematic->GetValue();
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
    // Keep a copy of the G-code the printer is running, for reference.
    const std::string orig_gcode = plate->get_tmp_gcode_path();
    try {
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

void LayerStreamDialog::on_save_tail()
{
    std::string gcode;
    if (!read_file(m_new_gcode_path, gcode)) {
        log(_L("Cannot read the revised G-code."));
        return;
    }
    ResumeOptions opts = resume_options();
    opts.set_kinematic_position = false; // Unknown without a printer; the file is for inspection or a manual start
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
                                          "LayerStream waits for the printer's next layer change, pauses and stops the current job, then "
                                          "uploads and starts the remaining layers of the revised model on %s. "
                                          "The bed and nozzle are reheated; Z is not homed."),
                                       m_diff.first_changed_layer + 1, m_diff.changed_z, from_u8(printer_cfg.opt_string("print_host"))),
                      _L("LayerStream"), wxYES_NO | wxICON_QUESTION);
    if (dlg.ShowModal() != wxID_YES)
        return;

    m_job = std::make_shared<ApplyJob>();
    m_job->thread = std::thread(run_apply, m_job, printer_cfg, m_new_gcode_path, m_diff.changed_z, resume_options());
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
