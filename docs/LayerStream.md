# LayerStream: revise a running print

LayerStream lets you change the model of a job that is already printing. The layers that are already printed stay as they are, and the
layers still to come are replaced with layers sliced from a revised model. It is meant for the case where you notice a design change while
the print is still below the changed part.

Open it from **File > Revise running print (LayerStream)…**.

## Flow

1. Keep the project, plate and settings exactly as they were when the job was sent. The plate's current slice is the reference for what
   the printer is printing.
2. **Choose revised model…** replaces the object's part with the revised file in place (the same mechanism as *Replace with 3D file*):
   the part keeps its transformation, and the file's own coordinates are mapped the same way as the original's. Then the plate is
   sliced again with the same configuration. Undo (Ctrl+Z) restores the original part.
3. **Plate position check.** The lower layers of both slices are compared (`register_on_plate`). If they coincide, the revised model sits
   exactly where the printed part is. If they coincide only after a translation, LayerStream offers to move the object by that offset
   and slices again. If no translation lines them up (rotated, scaled, or the base changed), applying is blocked with an explanation.
4. **Diff.** Per-layer object islands (`Layer::lslices`, every instance, world coordinates) are compared. The symmetric difference
   is opened by `xy_tolerance_mm` (0.02 mm) to drop slicing noise, and a layer counts as changed when more than 0.05 mm² remains. Changed
   layer heights, or a model that becomes taller or shorter, also count as changes.
5. **Preview.** The preview jumps to the first changed layer. An orange band marks its height across the object, and the changed regions
   are filled red.
6. **Apply changes to printer…** asks *Apply changes?* and then runs the printer sequence below. **Save tail G-code…** writes the
   same tail to a file without touching the printer, so you can inspect it first.

## Printer sequence (Creality stock firmware, e.g. Ender-3 V3 KE)

The printer's connection must be set to *Creality Print* with its IP address (Printer settings > Connection), the same setting that
*Send to printer* uses.

1. Read the status over the WebSocket. The job must be printing. The layer being printed comes from the firmware's `layer` counter,
   mapped to a G-code layer by counting the same marker lines the firmware counts (`LAYER_KEYS` in Creality's
   `klippy/extras/virtual_sdcard.py`: `;LAYER:`, `; layer:`, `; LAYER:`, `;AFTER_LAYER_CHANGE`, `;LAYER_CHANGE`) in the G-code the printer
   runs. The toolhead Z is only logged: on the KE profile the z-hop (0.2 mm) equals the layer height, so Z cannot tell layer n + hop
   from layer n + 1, and the pause macro lifts Z.
2. Wait for the counter to advance and pause at once, so the pause lands early in a layer.
3. Wait for paused, then read the counter again. The layer it names is the resume layer. It restarts from its beginning, so the few
   seconds of it printed before the pause are printed over. If that layer is past the changed layer, nothing more is sent.
4. Build the tail from the revised G-code: cut from the resume layer's `;LAYER_CHANGE` to the end, and put a resume preamble in front:
   - `M140`/`M104` (the stock `PAUSE` macro drops the nozzle to 140 °C).
   - `G90`, then an absolute `G1 Z<layer + 2>`, which is a move down from the pause park height and is clear of the part.
     An optional `G28 X Y` (off by default). Z is never homed. No `SET_KINEMATIC_POSITION`, no `G92 X/Y/Z`, no `SET_GCODE_OFFSET`:
     the paused job's position, Z offset and bed mesh are kept by the firmware and must stay as they are.
   - `M190`/`M109`. The active bed mesh is kept (a profile is loaded only if one is typed in the dialog).
   - Replayed `EXCLUDE_OBJECT_DEFINE` lines and the fan speed active at the cut.
   - Travel to above the tail's first XY, lower to the layer Z, and a small prime.
   - Restore the absolute or relative E mode, then `G92 E<value at the cut>` so the tail's absolute E values continue.
   - `find_forbidden_resume_command` rejects, in the preamble and the whole tail, bare `G28` or `G28` with Z, `G29`,
     `BED_MESH_CALIBRATE`, `PROBE`, `START_PRINT`, `FIRMWARE_RESTART`, `RESTART`, `G92` with X/Y/Z, `SET_GCODE_OFFSET` and similar.
5. Dry check (`check_tail`): logs the tail's first Z moves and the Z of its first extrusion, which must equal the resume layer Z. The
   same check runs in the dialog right after the comparison, before *Apply* is offered.
6. Upload the tail over HTTP without starting it and confirm it is in the printer's file list.
7. Switch the paused job: console G-code `CLEAR_PAUSE`, then `SDCARD_PRINT_FILE FILENAME=<tail>`. In Creality's Klipper the paused
   virtual_sdcard has no work timer, so `SDCARD_PRINT_FILE` is accepted. It closes the old file, loads the tail and runs only `G90`
   before the first line (homing is done only in the power-loss path). Then wait up to 45 s for `printing` with the tail's file name.
   If that does not happen, LayerStream reports it and stops. The original job is then normally still paused and can be resumed or
   cancelled on the printer screen.

The job is never stopped. On stock firmware a stop runs `CANCEL_PRINT`, which lifts Z by 2 mm, parks at X0 Y222, turns the heaters off
and runs `M84`, so the steppers are released and the position is lost. A tail started after that cannot be put back at the right height
without homing Z, which a part on the bed does not allow. The first real-printer test used stop and then declared the `curPosition`
reported after the stop with `SET_KINEMATIC_POSITION`, and printed at the wrong height.

## Stock Creality firmware protocol (as used here)

| Purpose | Transport | Message |
|---|---|---|
| Model / reachability | HTTP `GET http://<ip>/info` | JSON with `model` (Ender-3 V3 KE = `F005`) |
| Upload | HTTP `POST http://<ip>/upload/<name>` multipart `file` | stored under `/usr/data/printer_data/gcodes` on K1-family boards |
| Status | WebSocket `ws://<ip>:9999`, text frames | full dump on connect, then deltas; `{"method":"get","params":{"ReqPrinterPara":1}}` asks for it |
| Heartbeat | WebSocket | printer sends `heart_beat`, client answers `ok` |
| Start file | WebSocket | `{"method":"set","params":{"opGcodeFile":"printprt:<full path>"}}` |
| Pause / resume | WebSocket | `{"method":"set","params":{"pause":1}}` / `{"pause":0}` |
| Stop (not used: runs CANCEL_PRINT, M84) | WebSocket | `{"method":"set","params":{"stop":1}}` |
| Console G-code | WebSocket | `{"method":"set","params":{"gcodeCmd":"<line>"}}` |
| File list | WebSocket | `{"method":"get","params":{"reqGcodeFile":1}}` → `retGcodeFileInfo` |

Status keys used: `state` (1 printing, 5 paused, 4 stopped, 0 idle), `deviceState` (7 homing), `curPosition` (`"X:.. Y:.. Z:.."`),
`layer`, `TotalLayer`, `printProgress`, `printFileName`, `nozzleTemp`, `targetNozzleTemp`, `bedTemp0`, `targetBedTemp0`. The `layer`
value is virtual_sdcard's count of layer marker lines read so far. It runs a little ahead of the nozzle because of the motion queue.

The stock firmware has no Moonraker on port 7125. There is no documented way to replace the remaining part of a loaded job. LayerStream uses
the Klipper commands `CLEAR_PAUSE` and `SDCARD_PRINT_FILE` through the console G-code message, which Creality's own web server may or may
not pass through while a job is paused. A rooted printer with Moonraker gives the same commands through a supported API.

## Known limits

- The pause-and-switch sequence follows Creality's published Klipper source and the extracted KE macros. It has not run on a real KE yet,
  and Creality's closed web server, which handles the WebSocket, may reject `gcodeCmd` while paused.
- Klipper's idle timeout runs on a paused printer too. The switch takes seconds, but a paused job left for longer than the configured
  timeout loses its heaters and motors.
- The part of the resume layer printed before the pause is printed again.
- Only single-part objects can be revised, which is the same restriction as *Replace with 3D file*.
- Support or transition geometry under newly overhanging areas is generated by the normal slicer settings for the new layers. There is
  no dedicated chamfer yet.
