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

1. Read the status over the WebSocket. The job must be printing, and the reported toolhead Z must be below the first changed layer.
2. Wait for the next layer change, which is detected from `curPosition` Z when the same new layer is seen twice in a row, so that a Z
   hop is not mistaken for one. The tail restarts at the layer that has just begun, so only a few seconds of that layer get printed
   twice.
3. `pause` and wait for paused. Then `stop` and wait until the job is no longer busy. The stock firmware cannot load another file
   while a job is loaded, and it has no way to inject G-code into the running job.
4. Build the tail from the revised G-code: cut from the restart layer's `;LAYER_CHANGE` to the end, and put a resume preamble in front:
   - `M140`/`M104`, then `SET_KINEMATIC_POSITION X Y Z` set to the position reported after the stop. This is optional and needs
     Klipper's `force_move`, because the stop may disable the steppers.
   - A Z lift to clear the part, then an optional `G28 X Y`. Z is never homed.
   - `M190`/`M109`, then `BED_MESH_PROFILE LOAD=default`, and never a re-probe.
   - Replayed `EXCLUDE_OBJECT_DEFINE` lines and the fan speed active at the cut.
   - Travel to above the tail's first XY, lower to the layer Z, and a small prime.
   - Restore the absolute or relative E mode, then `G92 E<value at the cut>` so the tail's absolute E values continue.
   - `find_forbidden_resume_command` rejects `G28` with Z or bare `G28`, `G29`, `BED_MESH_CALIBRATE`, `PROBE`, `START_PRINT`,
     `FIRMWARE_RESTART` and similar commands.
5. Upload the tail over HTTP and start it, which reuses `CrealityPrint::upload` with *StartPrint*.

## Stock Creality firmware protocol (as used here)

| Purpose | Transport | Message |
|---|---|---|
| Model / reachability | HTTP `GET http://<ip>/info` | JSON with `model` (Ender-3 V3 KE = `F005`) |
| Upload | HTTP `POST http://<ip>/upload/<name>` multipart `file` | stored under `/usr/data/printer_data/gcodes` on K1-family boards |
| Status | WebSocket `ws://<ip>:9999`, text frames | full dump on connect, then deltas; `{"method":"get","params":{"ReqPrinterPara":1}}` asks for it |
| Heartbeat | WebSocket | printer sends `heart_beat`, client answers `ok` |
| Start file | WebSocket | `{"method":"set","params":{"opGcodeFile":"printprt:<full path>"}}` |
| Pause / resume | WebSocket | `{"method":"set","params":{"pause":1}}` / `{"pause":0}` |
| Stop | WebSocket | `{"method":"set","params":{"stop":1}}` |
| Console G-code | WebSocket | `{"method":"set","params":{"gcodeCmd":"<line>"}}` |
| File list | WebSocket | `{"method":"get","params":{"reqGcodeFile":1}}` → `retGcodeFileInfo` |

Status keys used: `state` (1 printing, 5 paused, 4 stopped, 0 idle), `deviceState` (7 homing), `curPosition` (`"X:.. Y:.. Z:.."`),
`layer`, `TotalLayer`, `printProgress`, `printFileName`, `nozzleTemp`, `targetNozzleTemp`, `bedTemp0`, `targetBedTemp0`. The layer
counter of the stock firmware is known to drift, so LayerStream derives the layer from Z.

The stock firmware has no Moonraker on port 7125. There is no supported way to replace the remaining part of a loaded job, so
LayerStream has to stop the job and start a new file, which loses the heated and homed state. A rooted printer that runs Moonraker could
keep the job alive (for example with `SDCARD_PRINT_FILE` while paused), but that is not implemented.

## Known limits

- Tested with unit tests and CI builds only. The printer sequence has not been run against a real printer yet.
- What the stock stop does (heaters off, park, steppers off) decides whether `SET_KINEMATIC_POSITION` or `G28 X Y` is needed. Both are
  options in the dialog.
- The bed cools a little between the stop and the reheat. Parts with weak bed adhesion may lift.
- Only single-part objects can be revised, which is the same restriction as *Replace with 3D file*.
- Support or transition geometry under newly overhanging areas is generated by the normal slicer settings for the new layers. There is
  no dedicated chamfer yet.
