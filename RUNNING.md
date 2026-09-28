# VeriMeasure (C++) — requirements and running procedures

This is the C++ port of the VeriMeasure measurement station. Behaviour is
unchanged from the Python original: same calibration model, same segmentation,
same sub-pixel edge refinement, same rectangle fit, same JSON file formats, same
keyboard controls, same operator page. If you have a `calibration.json` from the
Python version, this build reads it directly and vice versa.

Read the parity notes at the end before you sign anything off.

---

## 1. What you need

**Compiler.** Anything with working C++17. GCC 9 or newer, Clang 10 or newer,
MSVC 2019 16.8 or newer. Verified here on GCC 13.3.

**CMake** 3.16 or newer.

**OpenCV 4.2 or newer, or OpenCV 5.** On 4.x it needs `core`, `imgproc`,
`imgcodecs` and `calib3d`. On 5.x it needs `core`, `imgproc`, `imgcodecs` and
`geometry`, which is where OpenCV 5 moved the shape analysis and homography
functions that used to live in the first two. The build picks the right set
automatically. `highgui` is needed as well if you want the windowed tools; the
station and the offline harness build without it. Verified here on OpenCV 4.6.

**Arena SDK** from Lucid Vision Labs. This is not optional for a working
station. It is the only path to the camera, and this project has no substitute
frame source: nothing here fabricates, replays or simulates an image. A build
without it compiles and can still calibrate from a saved capture, but the
station and the live runner will refuse to start.

**Nothing else.** No Boost, no nlohmann, no Eigen, no web framework. JSON, the
HTTP server and the MJPEG stream are all in-tree, because asking a production
vision PC to pull down half a package index is a poor trade for code this small.

### Installing the prerequisites

Ubuntu and Debian:

```
sudo apt update
sudo apt install build-essential cmake libopencv-dev
```

Fedora and RHEL:

```
sudo dnf install gcc-c++ cmake opencv-devel
```

Windows: install Visual Studio with the C++ workload, CMake, and either a
prebuilt OpenCV or one you have built yourself. Point CMake at it with
`-DOpenCV_DIR=C:/opencv/build`.

The Arena SDK comes from the Lucid downloads page and needs a free account. On
Linux run the bundled `Arena_SDK_Linux_x64.conf` script after unpacking, which
registers the shared libraries and raises the network buffer limits. Do not skip
that step; the buffer sizes are the difference between a clean 20 MP stream and
one that drops most of its frames.

---

## 2. Building

From the project root:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

That gives you six binaries in `build/`:

| Binary | Purpose |
|---|---|
| `verimeasure_station` | the operator interface, served over HTTP |
| `verimeasure_cli` | the full pipeline with an OpenCV window |
| `phase1_camera_acquire` | camera connection and raw feed only |
| `phase2_roi_presence` | ROI plus background-difference presence test |
| `phase3_4_segmentation_measurement` | segmentation and pixel measurement |
| `offline_measure` | runs the engine over saved stills, no camera |
| `qc_check` | scores a folded edge against a style, no camera |
| `curve_check` | reads a pointed-at seam and scores it, no camera |
| `edge_accuracy` | how far the reported boundary sits from the drawn one |
| `collar_check` | finds a collar in a box on a saved frame, no camera |
| `perimeter_check` | reads a shirt's full perimeter in two halves, no camera |

Build `Release`, not `Debug`. The difference on the full-resolution refinement
pass is roughly an order of magnitude, and a `Debug` build will feel broken
rather than slow.

### Turning on the camera backend

**Do this.** A default configure leaves it off, and a build without it cannot
measure anything. CMake prints a warning when you have missed it.

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DVERIMEASURE_WITH_ARENA=ON \
      -DARENA_SDK_ROOT=/opt/ArenaSDK_Linux_x64
cmake --build build -j
```

Point `ARENA_SDK_ROOT` at the directory containing `include/Arena/ArenaApi.h`.
CMake locates the headers, the `libarena` shared object and the GenICam
libraries underneath it. On Windows the same flag works with the SDK's install
directory, typically `C:/Program Files/Lucid Vision Labs/Arena SDK`.

If the SDK is not compiled in, `verimeasure_cli` and `verimeasure_station` say so
and stop. That is deliberate. A measurement station that keeps running while
disconnected from its camera is worse than one that stops, because the numbers
on screen still look like measurements.

To work on something without occupying the rig, save a few frames off the camera
and use `--calibrate-image` and `offline_measure`. Those read real captures from
disk, which is the honest version of a test bench.

### Windows specifics

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

Binaries land in `build/Release/`, not `build/`. Winsock is linked
automatically. Put the OpenCV and Arena DLLs on `PATH` or beside the executables
before running.

Run the configure and build lines separately. Pasting both into PowerShell at
once makes the second a continuation of the first, so the build fires even when
configure has failed and you get two confusing errors instead of one.

### If CMake cannot find OpenCV

Linux drops a config file where CMake already looks, so this only comes up on
Windows, where OpenCV is wherever you happened to unpack it. The build script
checks the usual locations first: `C:/opencv/build`, a vcpkg install under
`C:/vcpkg`, a few others, and the `OpenCV_DIR`, `OPENCV_DIR` or `OPENCV_ROOT`
environment variables. When that works you will see a line like
`-- OpenCV found at C:/opencv/build` during configure.

If it does not, find the directory holding `OpenCVConfig.cmake`:

```powershell
Get-ChildItem C:\ -Filter OpenCVConfig.cmake -Recurse -ErrorAction SilentlyContinue |
  Select-Object -First 5 FullName
```

Then either pass it once:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=C:/opencv/build
```

or set it permanently so every future configure finds it on its own:

```powershell
[Environment]::SetEnvironmentVariable("OPENCV_DIR", "C:/opencv/build", "User")
```

Use forward slashes, or doubled backslashes. A single backslash is an escape
character to CMake and the path will silently come out wrong.

### If the build fails on contourArea, minAreaRect, moments or findHomography

You are on OpenCV 5 with a copy of this project from before it was made
version-aware. OpenCV 5 retired `calib3d` and moved shape analysis
(`contourArea`, `arcLength`, `boundingRect`, `minAreaRect`, `moments`,
`convexHull`, `getRotationMatrix2D`, `invertAffineTransform`) and
`findHomography` into a new `geometry` module. The functions are unchanged;
only the header and library that declare them moved.

The current version handles both layouts. `include/verimeasure/cv_compat.hpp`
includes the right headers based on `CV_VERSION_MAJOR`, and `CMakeLists.txt`
links `opencv_geometry` on 5.x and `opencv_calib3d` on 4.x. Configure prints
which one it chose. If you are seeing these errors, take the latest archive.

### LNK1104: cannot open file 'GCBase_MD_VC140_v3_3_LUCID.lib'

The headers were found and everything compiled; the linker just has nowhere to
look for the import libraries. On Windows the SDK headers auto-link by name
through `#pragma comment(lib, ...)`, so the names are fixed for us and only the
search path is ours to supply.

The current CMakeLists collects those directories itself, from `lib64/` and
`GenICam/` under `ARENA_SDK_ROOT`, and skips any 32-bit folder. Configure prints
what it found:

```
-- Arena library search paths:
--   C:/Program Files/Lucid Vision Labs/Arena SDK/lib64/Arena
--   C:/Program Files/Lucid Vision Labs/Arena SDK/GenICam/library/CPP/lib/Win64_i86
```

If that list is short or missing a GenICam entry, `ARENA_SDK_ROOT` is pointing
somewhere below the SDK root. It should be the folder that directly contains
`include/`, `lib64/` and `GenICam/`.

### The executables build but will not start

They need the Arena and GenICam DLLs at launch. The installer usually puts these
on the system `PATH`; if it did not, configure prints where they are:

```
-- Arena runtime DLLs (these must be on PATH at launch):
--   C:/Program Files/Lucid Vision Labs/Arena SDK/x64Release
```

Add them for the session:

```powershell
$env:PATH = "C:\Program Files\Lucid Vision Labs\Arena SDK\x64Release;$env:PATH"
```

Run the executables with a leading `.\`, or PowerShell treats the first path
segment as a module name and reports that it cannot load a module called
`build`:

```powershell
.\build\Release\phase1_camera_acquire.exe
```

### GenICam: "Using GenICam with untested compiler, version 1951 detected"

Informational, not an error. GenICam prints it whenever the MSVC version is
newer than the one it was validated against, which will be true for a while yet
on VS 2026. It does not stop the build and nothing is known to break because of
it. Ignore it.

A note on toolchains. Prebuilt OpenCV packages are compiled with the VS 2019 or
2022 toolset. Microsoft's binary compatibility guarantee normally carries that
onto a newer compiler without trouble, but if you hit link errors about
mismatched symbols, build OpenCV through vcpkg with your own toolset instead:

```powershell
git clone https://github.com/microsoft/vcpkg C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg install opencv4:x64-windows
```

then configure with
`-DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake`. vcpkg also
copies the DLLs next to the executables, which saves the `PATH` step.

---

## 3. Network and firewall for the camera

The TRI200S is a GigE Vision device and the usual failure modes are network, not
software.

Give the camera's NIC a static address on its own subnet, with nothing else on
it. Set the interface MTU to 9000 and confirm the switch, if there is one,
passes jumbo frames end to end. On Linux raise the receive buffers:

```
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
```

Persist those in `/etc/sysctl.conf` once you are happy. On Windows, allow
`verimeasure_cli.exe` and `verimeasure_station.exe` through the firewall on the
private profile, and enable jumbo frames in the adapter's advanced properties.

The tools print an estimated bandwidth figure at startup. If it is close to your
link rate, lower `stream_buffers` or accept a lower frame rate rather than
letting the driver drop partial buffers, which shows up later as an incomplete
count climbing while nothing appears to be wrong.

---

## 4. Running

### The operator station

```
./build/verimeasure_station
```

Open `http://localhost:8000`. To put the readout on a tablet or a second screen
at the bench:

```
./build/verimeasure_station --host 0.0.0.0 --port 8000
```

Then browse to the vision PC's address from the other machine. This serves on
your local network with no authentication, so keep it on the bench network.

Every frame on that page comes off the camera. There is no demo mode and no
synthetic source.

Options:

```
--host HOST              default 127.0.0.1, use 0.0.0.0 to expose it
--port PORT              default 8000
--results PATH           append every measurement to this JSON Lines file
--object-height-mm F     top-surface height above the mat, for parallax
--focal-length-mm F      the lens; saved in bench.json, so give it once. 0 forgets it
--camera-distance-mm F   camera to surface, measured with a tape; saved in bench.json
                         and kept through recalibration. 0 forgets it
--exposure-us F          the camera exposure by hand; saved in bench.json. 0 hands it
                         back to the station, which sets it from the clear mat
--exposure-target N      the level the clear base is metered to (default about 140);
                         saved in bench.json, used from the next Recalibrate. 0 forgets it
--forget-distortion      drop the lens distortion saved in bench.json (see "The lens's
                         distortion, measured once" below)
--no-capture             do not save a picture of each verdict
--capture-dir DIR        where the pictures go, default captures
--capture-quality N      their JPEG quality, default 85
--capture-keep-days N    keep the last N days of pictures; 0 (default) keeps all
--capture-max-gb F       keep at most F GB of pictures, oldest days first; 0 = no limit
```

**A picture of every verdict.** Each PASS and FAIL (and each step flagged for
manual QC) is saved as the feed showed it, with a band saying what was judged, to
`captures/PASS|FAIL|MANUAL/<date>/` (see section 5). Going Back to a step,
Retaking it or hovering its pip in the strip shows that step's last picture for
the shirt on the bench beside the feed, in the side panel; tap it to enlarge. It
is never drawn on the live feed, and a new shirt takes it off the page. The
pictures are drawn, encoded and written by a thread of their own: the measuring
thread only hands over the frame it already holds (a fraction of a millisecond,
logged on the console with the writer's time for each picture), and if the
writer falls eight pictures behind the oldest waiting one is dropped and said
in the log rather than hold up measuring. What is waiting is written before the
station exits on Ctrl-C.

**Give the lens once.** The fold-thickness correction needs the distance from the
lens to the mat, and the station works that out exactly from the focal length.
Started with `--focal-length-mm 12`, it saves 12 mm to `bench.json` beside
`calibration.json` and uses it on every start after, with or without the flag, and
through every recalibration. Without a lens it has to guess the distance from the
calibration, and that guess moved from 575 mm to 428 mm between two
recalibrations of the same bench, against a real 698 — enough to read the long
steps more than a millimetre short. The first line of the log says which it is
using.

**The exposure is set once, not on every start.** The first start with nothing
saved sets it from the picture, so start with the mat clear and the light on, and
saves it to `bench.json`. Every start after that uses the saved exposure as it
is, so the picture is equally bright every time. **Recalibrate**, which needs a
clear mat anyway, sets it again for the light as it is then; if the grid cannot
be read, the exposure goes back to what it was. It used to be metered afresh on
every start by a search that judged each step on the next frame. At two frames a
second that frame is usually one taken before the step, so the search stopped
anywhere from a third of the right exposure to three times it. On this bench one
start came up about ten times brighter than the next, with a white shirt 99.5%
clipped. The search now waits for each change to show before believing it.

The log says which exposure is in use. It warns when the picture is more than
2.5 times brighter or darker than when the exposure was saved, which usually
means the light has changed or is not on yet. To fix the exposure by hand, give
`--exposure-us` once: it is saved and Recalibrate leaves it alone until
`--exposure-us 0`.

**The metering target is the bench's to choose.** The clear base is metered so
its brightest content sits near 140 of 255, which leaves a white garment room
before it clips. A bench that measures dark cloth may want it brighter - black
cotton sits about 20 levels up at 140 - and `--exposure-target 180` saves that in
`bench.json`; it takes effect at the next Recalibrate, which meters the base
again. Every level added comes off a white garment's room: on the blue cloth a
white shirt already clipped over most of its area at about 140, and clipped cloth
blooms out over the base and reads long. `--exposure-target 0` goes back to 140.

The exposure keeps white cloth under the lamp below clipping, which leaves the
dark cutting mat looking nearly black. The **view** slider in the top bar lifts
the picture on that screen only, as a gamma so white stays white. It changes
nothing the camera takes or the station measures, and each browser remembers
its own setting.

**A measured working distance wins.** `--camera-distance-mm 700` saves the tape
reading to `bench.json`. From then on the fold-thickness correction uses it
instead of the lens figure, and a recalibration keeps it. The log still prints
the lens figure next to it, because the two disagreeing is how a printed grid
that is not true to size shows up. On the rigid mat they agreed to about 1%. A
WARNING at more than 1.5% means that, if the tape is right, readings from that
calibration are that much long or short.

The top bar has three controls besides the style: **Recalibrate**, **Check
scale** and the view slider. Beside them it says whether the scale has been
checked against a reference ("scale checked" in green) or not yet (amber).

**Recalibrate** retakes the empty base: it sets the camera's exposure for the
light as it is, waits for the base to hold still, and takes a fresh
empty-surface reference (the background). What happens to the *geometry* - the
model that turns pixels into millimetres - depends on the base:

- **On the rigid cutting mat** (bright lines on a dark mat) the printed grid is a
  ruler. It is read and fitted, and the fit becomes the geometry, as it always
  has.
- **On the blue cloth** (dark lines on a light ground, the bench's base since
  September 2026) the grid is *not* a ruler. Clamped and creased, its squares
  measured 1.25% long across the middle and up to 2.8% toward the edges against
  the rigid mat, and a creased one does not fit a plane at all. So on the cloth
  Recalibrate keeps the geometry already in use - with its checked scale - and
  takes only the exposure, the background and the working area from the cloth.
  It no longer fails because the cloth's grid will not fit, and it no longer
  replaces a good geometry with a stretched one. The card says which geometry
  was kept.

Which way round the grid is printed is decided from the middle of the picture
on every recalibration. To let a dark-lined grid set the geometry anyway, set
`dark_grid_sets_geometry` in `config.hpp`; with no geometry at all to keep (no
`calibration.json`), it always does, with a warning.

The geometry itself therefore comes from the rigid mat. After moving the
camera, lay the rigid mat, press Recalibrate, then put the cloth back and press
it again. Then check the scale.

**The lens's distortion, measured once.** A recalibration fits the homography
(where the camera and the mat are) and two radial terms (the lens). Fitted
afresh from one picture, the radial terms are pinned only by whichever grid
points near the edges that picture happened to read: six recalibrations of the
unmoved rig minutes apart put the same pixel up to 13 mm apart at the edges of
the frame and 16 mm in the corners, against half a millimetre in the middle. The
lens does not change between recalibrations, so it can be measured once, from
several pictures, and kept:

1. Lay the rigid mat, clear, and do not touch the camera.
2. Save 5 to 8 pictures of it (the station's `dump`, or any still off the
   camera), moving the mat a few centimetres - and turning it a little -
   between them. Moving it is the point: a misprinted square then lands in a
   different place in each picture instead of being fitted into the lens.
3. `verimeasure_cli --fit-distortion p1.png p2.png ... p8.png` from the
   station's working directory. It prints, for each picture, how well one
   shared distortion fits it, and how far calibrations of the pictures agree
   with each picture's own distortion and with the shared one. If it says the
   mat did not seem to move, move it and take the pictures again.
4. Happy with it, run the same line again with `--save`. The distortion goes
   into `bench.json`, and from then on every recalibration - the station's
   Recalibrate included - fits only the homography, and says so
   (`"distortion_source": "bench"` in `calibration.json`).
5. Press Recalibrate on the rigid mat.

`--forget-distortion` (station or CLI) drops it again; with none saved,
recalibration is exactly what it was. Refit it after refocusing the lens or
changing the aperture, which changes the distortion; moving the camera does
not.

**Recalibrate waits for the base to be clear.** It used to take the background
from whatever was lying there, and a garment left on the base became part of
the "empty" reference, invisible to every measurement after it. Now, pressed
with something on the base, the card says *Lift everything off the base* and
how much of the printed grid is hidden, and recalibrating starts by itself the
moment the base is clear. **Cancel** on the card calls it off with nothing
changed. After a minute it gives up and says so. "Clear" is judged by the
printed grid: squares of the working area where the lines no longer stand out
from the cloth or mat around them. On the bench's frames a clear base hid none
of about 900 squares, and a garment hid a fifth to three quarters of them. The
background is checked the same way once it is taken, and the station's start
warns if something was lying on the base when it started.

**Did the camera move?** A kept geometry is only as good as the camera's
position, so each recalibration on the cloth compares the cloth's printed pitch
with the last one's. If the squares look larger or smaller by more than 0.5%
both ways alike - what raising or lowering the camera does - the card warns how
far off readings would now be and says to check the scale. A change in one
direction only is what re-stretching the cloth does, and it says so.

Recalibrate refuses to run in the dark. If the exposure search reaches the
camera's longest exposure and the picture is still well short, the lamp is off.
The grid might still be read, but the calibration would be saved with an
exposure several times too long for the lamp. The card says to turn the light
on, and the calibration and exposure stay as they were.

A recalibration that replaces the geometry (the rigid mat) says how differently
it measures from the one it replaced, across the middle and toward the edges,
keeps the replaced one as `calibration.prev.json`, and warns when the difference
is more than two calibrations of one rigid mat ever show (0.5% across the
middle, 1.5% at the edges). To go back, stop the station, copy
`calibration.prev.json` over `calibration.json`, and start it again.

A success that comes with something to do - the scale not yet checked, the
camera possibly moved, a residual above the limit - stays on screen in amber
until **OK** is pressed. A plain success clears itself after a few seconds.

**Check scale** is what makes readings true on a base that is not a ruler. The
geometry says how the picture maps onto a plane; only a reference of known
length says the plane is where it was measured. The cloth lies a mat's
thickness lower than the mat it replaced, and a camera raised a few millimetres
reads everything short by the same fraction. To check it:

1. Press **Check scale**. Nothing is judged while the card is open - the rule is
   not a garment.
2. Lay a steel rule flat in the middle of the base, on its own. The longest one
   that fits is best: 500 to 600 mm halves the effect of a tenth of a
   millimetre at each end compared with 300 mm.
3. When the card shows its reading, type its true length end to end (the whole
   rule, not the graduated part) and its thickness (1 mm for most steel rules).
   The camera sees the rule's top face, so it is read at that height, not at the
   fold height of whatever step is showing.
4. Better, before applying: lift the rule and lay it again toward the left of
   the base, and toward the right. The card reads each place (centre, left,
   right, top, bottom, by where the rule lies) and keeps the latest reading in
   each; laying it again in the same place replaces that place's reading.
5. The card shows what the correction would be. Press **Apply**. Every reading
   from then on is corrected by the same factor, which is saved in
   `calibration.json` with the length, the reading, the thickness and the time,
   and kept through every recalibration on the cloth. Read in more than one
   place, the places have to ask for the same correction to within 0.3%
   (`scale_check_agree`), and then it is their average, with each reading used
   listed under `scale_verified_places`. If they do not agree, nothing is
   applied and the card says what each place asked for - "centre -0.06%, right
   -1.14%". A scale error is the same everywhere; places that disagree are the
   lens or the corners of the picture, which one factor cannot mend (see "The
   lens's distortion, measured once").
6. **Close** returns to the steps.

A correction beyond 5% is refused: that is a length typed wrong, something else
read with the rule, or a geometry that no longer fits the camera - lay the
rigid mat and press Recalibrate. Past 2% the card asks for the length to be
checked before applying.

The `clear` command (`POST /api/command?name=clear`, no button on the page) empties
the on-screen log and starts again at shirt 1.

### The command line runner

```
./build/verimeasure_cli
```

Opens an OpenCV window. Keys, with the window focused:

| Key | Action |
|---|---|
| `c` | recalibrate the plane model from the current empty surface; on a dark-lined grid (the cloth) only the background is retaken and the geometry kept, as on the station |
| `b` | recapture the background reference |
| `v` | verify and correct global scale against a known artefact |
| `r` | draw the ROI by hand instead of using the lattice footprint |
| `m` | toggle the mask view |
| `q` | quit |

Options:

```
--headless               no windows, results to stdout as JSON Lines
--recalibrate            ignore any saved calibration and fit a new one
--manual-roi             draw the ROI by hand at startup
--calibrate-image PATH   calibrate from a still image and exit
--results PATH           append results to this JSON Lines file
--object-height-mm F     top-surface height above the mat
--focal-length-mm F      the lens; saved in bench.json, shared with the station
--exposure-target N      the level the clear mat is metered to; saved in bench.json
--fit-distortion P...    measure the lens's distortion from several pictures of the
                         rigid mat and exit; with --save, keep it in bench.json
--forget-distortion      drop the saved distortion
```

Pressing `v` prompts on the terminal, so keep the console visible if you intend
to use it.

### The phase tools

These are the original diagnostic steps, kept intact. Reach for them when the
station will not start and you need to find out where the chain breaks.

```
./build/phase1_camera_acquire                    # q to quit
./build/phase2_roi_presence                      # b background, q quit
./build/phase3_4_segmentation_measurement        # b background, m mask, q quit
```

Phase 1 answers "is the camera reachable and streaming". Phase 2 answers "does
the background difference see an object". Phase 3+4 answers "does segmentation
produce a sane contour". Work upward from whichever one fails.

### Working from saved captures

These read real frames previously taken off the camera. They exist for
reproducing a fault and for validating a change without tying up the rig; they
are not a substitute camera.

Calibrate from a saved photograph of the empty mat:

```
./build/verimeasure_cli --calibrate-image mat.png
```

Measure from saved stills:

```
./build/offline_measure background.png part.png calibration.json
```

That prints one JSON object, the same record the live pipeline emits. It is the
fastest way to reproduce a fault someone reported, and it is how this port was
checked against the Python original.

Read a collar from a saved frame, from a box given in pixels of the frame
(see [docs/collar_detection.md](docs/collar_detection.md)):

```
./build/collar_check --frame lay.png --box 1800,1850,1430,870 --calibration calibration.json --draw out.png
./build/collar_check --case collar-fail.json --draw out.png
./build/collar_check --selftest
```

The second replays exactly what a station saw when it could not read a collar;
the station writes that case and prints the command line when it happens. Add
`--around` to either to read the whole collar - the outer edge over the front and
the edge of the shirt across the back of the neck - as a step with
`"measure": "around"` does; the tester style's collar step measures that way.
On the feed the whole collar is drawn in green and the opening's edge dotted.

Check the full-perimeter step, or replay a half a station could not read
(see [docs/garment_perimeter.md](docs/garment_perimeter.md)):

```
./build/perimeter_check --selftest
./build/perimeter_check --case perimeter-fail.json --draw out.png --verbose
```

---

## 5. Files it reads and writes

All paths are relative to the working directory, so run from wherever you want
these to live.

| File | Written by | Contents |
|---|---|---|
| `calibration.json` | calibration, Recalibrate, Check scale | homography, radial terms, scale correction and what it was checked against, fit metadata, and (`meta.base`) the printed pitch of the working base when last recalibrated on it |
| `roi_config.json` | ROI selection | the measurement rectangle in full-resolution pixels |
| `calibration.prev.json` | Recalibrate on the rigid mat | the calibration the last recalibration replaced, to go back to. A recalibration on the cloth replaces nothing and leaves it alone |
| `bench.json` | `--focal-length-mm`, `--camera-distance-mm`, `--exposure-us`, `--exposure-target`, `--fit-distortion --save`, the first start, Recalibrate | the lens, the measured working distance, the camera exposure with the highlight level it gave and whether it was set from the mat or by hand, the level the base is metered to, and the lens's radial distortion (`distortion`: the two terms, their centre and radius, the picture size, and the fit it came from) |
| whatever you pass to `--results` | every measurement | JSON Lines, one record per line |
| `collar-fail.png`, `collar-fail.json` | a collar step that could not be read | the held frame, the box, the outline and the step, for `collar_check --case`; overwritten each time |
| `perimeter-fail.png`, `perimeter-fail.json` | a half of a perimeter step that could not be read | the frame, the outline, the working area and the clip colour, for `perimeter_check --case`; overwritten each time |
| `captures/PASS\|FAIL\|MANUAL/YYYY-MM-DD/HHMMSS_g<shirt>_s<step>_<style>.jpg` | the station, for every verdict | the picture as the feed showed it when the verdict was filed (outline, span, seam, distance or collar), under a band with the verdict, the step, the reading against its target, the style, the shirt and the time. A retake is a new file (`_2`, `_3` in the same second); nothing is overwritten. The record in `--results` names it in `"image"`. `--no-capture` turns it off; `--capture-dir`, `--capture-quality` (default 85), `--capture-keep-days` and `--capture-max-gb` (0 = no limit, the default) set where, how well and how much is kept. Retention removes whole days, oldest first, and only ever `*.jpg` directly inside those date folders |

The schema is version 2 and byte-compatible with the Python original, including
key order. Copy your existing files across and this build picks up where the old
one left off. The pair that shipped with the Python project is in `config/` for
convenience; copy them into your working directory to skip the first
calibration, or delete them and let the rig fit its own.

---

## 6. Commissioning procedure

Do this in order the first time, and again after anything moves.

1. **Clear the surface.** Nothing on the mat, no hands in frame, lighting at its
   working level. Calibration reads the printed grid, so anything covering it
   costs you points.
2. **Start the station or the CLI.** It will lock the camera's auto adjustments,
   set exposure, wait for the scene to settle, capture a background reference,
   then detect the grid and fit the plane model.
3. **Check the residual.** It is printed at startup and shown on the page. Under
   0.1 mm on a good mat is normal. Above 0.35 mm the fit is rejected outright.
   If it is marginal, look for glare on the mat, a fold in it, or a lens that
   needs refocusing.
4. **Check the scale against a real artefact.** This is the step people skip and
   it is the one that matters. A self-healing cutting mat is a drawing aid; print
   scale on these is commonly off by half a percent to two percent, and not
   uniformly - and the bench's printed cloth is further off than that. On the
   station press **Check scale**, lay a steel rule, let it read, and enter its
   true length end to end and its thickness (see "The operator station"). On the
   CLI, let it measure the rule and press `v`. One global scale factor is stored,
   and it is the difference between repeatable and correct. The station refuses
   a correction beyond 5%, the CLI beyond 10%: at that point you have a
   calibration fault rather than a scale error.
5. **Measure a second known part** to confirm the correction holds. If it does
   not, the plane model is wrong, not the scale.

The scale correction survives in `calibration.json` with the artefact length and
a timestamp, so an audit can see what it was verified against and when.

---

## 7. Parity with the Python original

The port was checked end to end, not assumed correct.

**Calibration**, over the same synthetic mat: identical line detection (49
horizontal, 72 vertical), identical indexing, identical lattice counts (3,277
detected, 3,249 used), identical keep-or-drop decision on the distortion terms,
identical footprint. Full-fit RMSE came out at 0.020840 mm in Python against
0.020855 mm here, and `mm_per_pixel` agrees to about one part in a million.

**Measurement**, over four parts from 85 mm to 297 mm, using frames rendered
once and fed to both codebases as files: length and breadth agree
to around 1e-4 mm, roughly a tenth of a micron on an A4-sized sheet. The fit
method chose `edge_fit` in every case in both. Side-fit RMS agrees to 1e-7 mm.

Several things were preserved deliberately rather than written the obvious way,
because the obvious way shifts results:

- Median over an even count averages the middle pair, as NumPy does.
- Percentiles use linear interpolation between order statistics, as NumPy does.
- Float to `uint8` conversion **truncates**. OpenCV's `convertTo` rounds. A
  one-level shift moves the Otsu threshold, so the truncating path is used.
- Rounding is `nearbyint`, which is banker's rounding under the default mode and
  matches Python.
- Segmentation arithmetic stays in float32 rather than being promoted.
- The weighted quadratic line fits go through SVD, mirroring `lstsq` rather than
  a normal-equations solve.

### The one deliberate deviation

`fit_plane_model` splits the lattice into a training set and a holdout set to
decide whether the two radial distortion terms earn their place. Python draws
that permutation from NumPy's PCG64 seeded at 20260831. This build uses a
fixed-seed Mersenne shuffle instead.

Different points therefore land in the holdout set and the reported holdout RMSE
differs (0.535 mm against 0.503 mm on the test mat). This does not affect
measurement. The permutation only feeds a single boolean, "keep the distortion
terms or not", which came out the same in every case tried; and whichever
structure wins is then refitted on **all** points, so no measuring model is ever
built from a subset. Reproducing PCG64 along with NumPy's bounded-integer
rejection sampling would have been a large amount of untestable code for no
measurable gain.

That is the only deviation. It is commented at the point of use in the source.

### One thing to confirm on your machine

`angle_deg` comes straight from OpenCV's `minAreaRect`, which labels a rectangle
by one of its two edge directions. Between OpenCV versions that label can land on
the other edge, so the value shifts by 90 degrees for the same physical
rectangle. Length and breadth are taken as the max and min of the pair and are
unaffected, which the parity run confirms.

The test above compared a C++ build on OpenCV 4.6 against a Python install on
4.13, so that column shows the offset. Built against matching OpenCV versions it
agrees as well. Worth a quick check on your rig if downstream consumes the angle.

---

## 8. When something goes wrong

**No frames after starting the stream.** Network, almost always. Check the MTU,
the receive buffers, and that only the camera is on that subnet. Arena's own
IpConfigUtility will tell you whether the camera is even visible.

**Incomplete count climbing.** Bandwidth. Lower `stream_buffers`, raise the
packet delay in Arena, or accept a lower frame rate.

**Calibration fails on a clear mat.** The grid is not being read. Look for glare,
a shallow lighting angle that flattens the printed lines, or a defocused lens.
`--calibrate-image` on a saved frame lets you iterate on this without tying up
the rig. On the cloth this no longer happens: its grid is not fitted while there
is a geometry to keep. It only fails there with no `calibration.json` at all -
lay the rigid mat for that first calibration.

**"Lift everything off the base".** Recalibrate was pressed with something on the
base, or something the grid cannot be seen through: a garment, a hand, a sheet
of paper. It starts by itself once the base is clear; Cancel calls it off. If the
base really is clear and it still waits, the printed grid cannot be seen in the
part of the picture it names - out of focus, or glare burning the lines out.

**"The base's printed squares look +x% ...".** The cloth's pitch changed since the
last recalibration. Both ways alike means the camera probably moved up or down,
and readings are off by about that much until the scale is checked (Check scale)
or the rigid mat is recalibrated. One way only means the cloth was moved or
re-stretched, which changes nothing measured.

**"Too dark even at the camera's longest exposure".** The lamp is off, or much
dimmer than when the exposure was set. Turn it on and press Recalibrate again.

**Readings moved by a per cent or two after a recalibration.** Read the
recalibration's own report in the log. If it says the new calibration measures
differently from the last one and the camera has not moved, the new surface's
printed grid is not true. Go back to `calibration.prev.json`, or keep the new
one and correct it with a steel rule and Check scale. This is what happened when
the cloth first went down on 2026-09-23: calibrated on the cloth, readings came
out 10-12 mm long. The rigid mat's geometry was put back (the cloth ones are
kept beside it as `calibration.cloth-*.json`) and the cloth now only gives the
background.

**Measurements repeatable but wrong by a consistent percentage.** You have not
run the scale check in step 4, or the camera has moved since. Do that.

**"Not measured: the outline found runs along N sides of the working area".**
The outline of what settled is the edge of the picture: the base itself is being
read as the object. Most often something covers nearly the whole view, or the
background was taken with something on the base, or the light has changed a
great deal since it was. Nothing is measured while it lasts. Check the base and
press Recalibrate with it clear. A garment wider and taller than the view - a
fold at the rail with the rest of the shirt running off the top and both sides -
touches three sides as well and is refused the same way; if lays like that are
measured on this bench, set `outline_refuse_sides` to 4 in `config.hpp`.

**A dark garment covering more than half the view.** Handled: the brightness
correction between the picture and the background is read from the base alone
when the whole view reads much darker than the background. Read from the whole
view, a black shirt covering half of it used to turn the whole base into the
object and the outline into the frame. A pale garment keeps the whole-view
correction, as it always has: on the blue cloth a white shirt lights the cloth
beside it, and read from the base alone that glow was taken for cloth and the
outline ran several millimetres out (`gain_mat_only_brighter` in `config.hpp`).

**A dark garment reading short, its corners cut.** Handled: a black shirt on the
blue cloth used to lose 5 to 7 mm along every edge and a bite out of its
corners, so every step read several per cent short. Shadows are recognised by
the base's pattern showing through them, and dark pixels with no pattern of
their own are voted shadow by the shadow around them - which assumed no cloth is
darker than the base. With a garment darker than the base under its outline,
a pixel where the base's pattern should show and does not now stays garment
(`dark_garment_keeps_evidence`). A pale garment never takes this path.

**The outline of a black shirt bulging onto the base, or biting into the shirt.**
Handled. The pattern test above has a blind side: the shirt's shadow across a
printed line, or a line the base has crept off since it was taken, is missing
its pattern too, and the outline ran out along those lines 3 to 4 mm; between
two lines, where there is no pattern to test, the shirt was still cut away a
grid cell at a time. A garment near black - its middle brightness under
`dark_garment_black_share` (15%) of the base's - is now told from its shadow by
how dark it is: a pixel as dark as the cloth is cloth, and nothing else gets the
benefit of the doubt. On rendered black lays with a known outline this took the
outline error from 3.6-3.9 mm RMS (9 mm cuts) to 0.26-0.30 mm (1.2 mm at
worst, never more than 0.4 mm out). A navy or grey garment, and every pale one,
is segmented exactly as before. `dark_garment_black_share = 0` turns it off.

**The outline running out along the printed grid, in steps.** Handled. The cloth
moves under a garment as it is laid - 4 to 19 px beside one sleeve on 2026-09-25,
up to 29 px on the white shirt of 2026-09-23 - and every printed line it moves is
a difference from the background, which the outline was joined to wherever it
touched the garment. The fine pass now reads how far the base has moved, on the
bare cloth round the lay, and measures against the background moved to match; the
log says "the base had moved since the background was taken (up to ... px)". On
that sleeve the outline went from 3.6 mm to 0.3 mm off the shirt's edge on
average. Where the base has not moved nothing changes. If the log shows it on
every lay with tens of pixels, the cloth is sliding: clamp it flatter.
`base_creep_align = false` in `config.hpp` turns it off. `dump` also writes the
whole working area - `frame_full.png` and `background_full.png` - so a lay can be
looked at afterwards.

**A collar on a black shirt not found.** The collar's edges are one or two gray
levels deep on black cloth, under every floor the reader holds a line to, so
nothing of the collar was seen. Below `collar_dark_below` (24 gray levels) of
cloth in the box the collar is now read from a picture scaled until the cloth
sits at `collar_dark_level` (80). A pale collar is read from the same picture as
before. What this cannot do is read a polo collar as a band: the reader looks for
a crew-neck rib band 8 to 35 mm wide running unbroken from shoulder to shoulder,
and a polo's collar lies as a leaf about 50 mm wide, broken at the placket. On a
polo it offers the line it did find for a tap (see docs/collar_detection.md) -
check where that line runs before tapping it.

**The white outline sitting just inside the cloth on a fold step.** Fixed, on
the feed only. A step with `height_mm` has the fold's height taken off the
outline itself, as a magnification about the lens axis, so that it measures the
cloth's top face; that outline was also what was drawn, which put the line up to
three screen pixels inside the cloth toward the edges of the picture. The feed
now draws the edge where it is in the picture. The reading is unchanged.

**"Not applied: the reference asks for a different correction in each place".**
Check scale was read in more than one place and the places disagree by more than
0.3%. That is not a scale error but the lens or the corners of the picture.
Measure the lens's distortion (see "The lens's distortion, measured once"),
recalibrate on the rigid mat, and check again.

**Clipped-white warning on results.** Part of the object is saturated and
blooming inflates its edges outward. Reduce the exposure: give a lower
`--exposure-us` (the log shows the one in use) and restart with the mat clear,
or turn the light down and press Recalibrate with the mat clear.

**The picture is bright on one start and dark on the next.** That was the
startup exposure search, fixed as described under "The operator station". If it
still happens, check the log's first lines: a warning that the picture is much
brighter or darker than when the exposure was saved means the light itself
changed.

**Parts joining together in the mask.** They are closer than the gap-bridging
distance, which is sized at 30 percent of the detected grid pitch. Separate them
on the mat; the bridging exists to close printed lines and cannot be widened
without joining neighbours.
