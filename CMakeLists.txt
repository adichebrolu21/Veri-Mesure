cmake_minimum_required(VERSION 3.16)
project(VeriMeasure LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

# The Arena SDK is what connects this software to the camera. Turn this ON for
# any build that will actually measure something. With it OFF the project still
# compiles, and you can still calibrate from a saved capture, but the station
# and the live runner will refuse to start: there is no substitute frame source
# and nothing in this project fabricates an image.
option(VERIMEASURE_WITH_ARENA "Build against the Lucid Arena SDK" OFF)
set(ARENA_SDK_ROOT "" CACHE PATH "Root of the Arena SDK installation")

# ---------------------------------------------------------------------------
# Locating OpenCV
# ---------------------------------------------------------------------------
#
# On Linux the system package drops a config file where CMake already looks, so
# none of this runs. Windows has no such convention: OpenCV is wherever you
# unpacked it, and CMake has no way to guess. Rather than make everyone pass
# -DOpenCV_DIR on every configure, we look in the places it usually lands.
#
# Order of precedence:
#   1. -DOpenCV_DIR=... on the command line
#   2. the OpenCV_DIR or OPENCV_DIR environment variable
#   3. an OPENCV_ROOT / OpenCV installation discovered below
#
# The target is the directory holding OpenCVConfig.cmake. In a prebuilt Windows
# package that is <root>\build; in a vcpkg or self-built install it is usually
# <prefix>/share/opencv4 or <prefix>/lib/cmake/opencv4.

if(NOT OpenCV_DIR)
  foreach(_env_var OpenCV_DIR OPENCV_DIR OPENCV_ROOT)
    if(DEFINED ENV{${_env_var}} AND NOT OpenCV_DIR)
      set(OpenCV_DIR "$ENV{${_env_var}}" CACHE PATH "Directory containing OpenCVConfig.cmake")
      message(STATUS "OpenCV_DIR taken from the ${_env_var} environment variable: ${OpenCV_DIR}")
    endif()
  endforeach()
endif()

if(NOT OpenCV_DIR)
  # Candidate roots. Add your own here if OpenCV lives somewhere unusual and you
  # would rather not repeat the flag on every configure.
  set(_opencv_hints
    "C:/opencv/build"
    "C:/opencv"
    "C:/tools/opencv/build"
    "C:/Program Files/opencv/build"
    "C:/Program Files (x86)/opencv/build"
    "C:/vcpkg/installed/x64-windows/share/opencv4"
    "C:/vcpkg/installed/x64-windows/share/opencv"
    "C:/dev/opencv/build"
    "C:/libs/opencv/build"
    "$ENV{USERPROFILE}/opencv/build"
    "$ENV{USERPROFILE}/Downloads/opencv/build"
    "/usr/local/lib/cmake/opencv4"
    "/usr/local/share/opencv4"
    "/opt/opencv/lib/cmake/opencv4"
  )

  find_path(VERIMEASURE_OPENCV_CONFIG_DIR
    NAMES OpenCVConfig.cmake opencv-config.cmake
    HINTS ${_opencv_hints}
    PATH_SUFFIXES "" build x64/vc17/lib x64/vc16/lib x64/vc15/lib
                  share/opencv4 lib/cmake/opencv4
    NO_DEFAULT_PATH
  )

  if(VERIMEASURE_OPENCV_CONFIG_DIR)
    set(OpenCV_DIR "${VERIMEASURE_OPENCV_CONFIG_DIR}"
        CACHE PATH "Directory containing OpenCVConfig.cmake" FORCE)
    message(STATUS "OpenCV found at ${OpenCV_DIR}")
  endif()
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
#
# No version is requested here, deliberately. OpenCV's config file treats a
# version request as same-major-version, so asking for "4" is refused outright
# by an OpenCV 5 installation and vice versa. The version is checked below
# instead, once we can see what is actually installed.
#
# No component list either. A "world" build ships one library and does not
# always answer component queries, and the module names differ between 4 and 5
# anyway. Taking OpenCV_LIBS wholesale works with every layout.

find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
  message(FATAL_ERROR
    "OpenCV was not found.\n"
    "\n"
    "OpenCV_DIR is currently: ${OpenCV_DIR}\n"
    "\n"
    "This project needs the directory that contains OpenCVConfig.cmake. Find it\n"
    "with one of these, depending on your shell:\n"
    "\n"
    "  PowerShell:\n"
    "    Get-ChildItem C:\\ -Filter OpenCVConfig.cmake -Recurse -ErrorAction SilentlyContinue |\n"
    "      Select-Object -First 5 FullName\n"
    "\n"
    "  Linux or macOS:\n"
    "    find / -name OpenCVConfig.cmake 2>/dev/null | head -5\n"
    "\n"
    "Then pass it in, using forward slashes:\n"
    "    cmake -S . -B build -DOpenCV_DIR=C:/opencv/build\n"
    "\n"
    "Or set OPENCV_DIR once in your environment and it will be picked up\n"
    "automatically on every configure from then on.\n"
    "\n"
    "Delete the build directory before retrying. A cache from a failed\n"
    "configure will otherwise hold the old result.")
endif()

if(OpenCV_VERSION VERSION_LESS 4.2)
  message(FATAL_ERROR
    "OpenCV ${OpenCV_VERSION} was found at ${OpenCV_DIR}, but this project needs "
    "4.2 or newer. OpenCV 5 is supported.")
endif()

message(STATUS "Using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

# OpenCV 5 retired calib3d and moved the shape-analysis and homography
# functions into a new "geometry" module. The source side of the same split is
# handled in include/verimeasure/cv_compat.hpp. Nothing to do here beyond
# reporting it, since OpenCV_LIBS already carries whichever one exists.
if(OpenCV_VERSION_MAJOR GREATER_EQUAL 5)
  message(STATUS "  shape analysis and homography come from the geometry module")
else()
  message(STATUS "  shape analysis and homography come from imgproc and calib3d")
endif()

find_package(Threads REQUIRED)

# highgui is only needed by the three windowed tools.
if(TARGET opencv_highgui OR "opencv_highgui" IN_LIST OpenCV_LIBS
   OR "opencv_world" IN_LIST OpenCV_LIBS)
  set(VERIMEASURE_HAVE_HIGHGUI ON)
else()
  set(VERIMEASURE_HAVE_HIGHGUI OFF)
  message(STATUS "OpenCV highgui not found: the windowed phase tools will be skipped")
endif()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------

add_library(verimeasure STATIC
  src/json.cpp
  src/numeric.cpp
  src/plane_model.cpp
  src/calibration.cpp
  src/segmentation.cpp
  src/creep.cpp
  src/measure.cpp
  src/edge.cpp
  src/curve.cpp
  src/collar.cpp
  src/perimeter.cpp
  src/finger.cpp
  src/template.cpp
  src/qc.cpp
  src/engine.cpp
  src/camera.cpp
  src/http_server.cpp
)

target_include_directories(verimeasure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(verimeasure PUBLIC
  ${OpenCV_LIBS}
  Threads::Threads
)

target_include_directories(verimeasure PUBLIC ${OpenCV_INCLUDE_DIRS})

if(WIN32)
  target_link_libraries(verimeasure PUBLIC ws2_32)

  # windows.h defines min and max as function-like macros, which collide with
  # std::min and std::max and produce C2589 "illegal token on right side of ::".
  # NOMINMAX suppresses exactly those two macros and nothing else; it does not
  # touch any other part of the Windows headers. Applied to the whole target so
  # the ordering of ArenaApi.h and winsock2.h stops mattering.
  target_compile_definitions(verimeasure PUBLIC NOMINMAX)
endif()

if(VERIMEASURE_WITH_ARENA)
  if(NOT ARENA_SDK_ROOT)
    message(FATAL_ERROR "Set -DARENA_SDK_ROOT=<path> when VERIMEASURE_WITH_ARENA is ON")
  endif()

  find_path(ARENA_INCLUDE_DIR ArenaApi.h
    HINTS "${ARENA_SDK_ROOT}/include/Arena" "${ARENA_SDK_ROOT}/include"
    NO_DEFAULT_PATH)
  find_path(GENICAM_INCLUDE_DIR GenICam.h
    HINTS "${ARENA_SDK_ROOT}/GenICam/library/CPP/include"
          "${ARENA_SDK_ROOT}/include/GenICam"
    NO_DEFAULT_PATH)
  find_path(SAVE_INCLUDE_DIR SaveApi.h
    HINTS "${ARENA_SDK_ROOT}/include/Save" NO_DEFAULT_PATH)

  if(NOT ARENA_INCLUDE_DIR)
    message(FATAL_ERROR "ArenaApi.h not found under ARENA_SDK_ROOT=${ARENA_SDK_ROOT}")
  endif()

  find_library(ARENA_LIBRARY
    NAMES arena arenac Arena_v140 Arena_v143
    HINTS "${ARENA_SDK_ROOT}/lib64" "${ARENA_SDK_ROOT}/lib"
          "${ARENA_SDK_ROOT}/lib64/Arena")
  if(NOT ARENA_LIBRARY)
    message(FATAL_ERROR "Arena library not found under ARENA_SDK_ROOT=${ARENA_SDK_ROOT}")
  endif()

  target_compile_definitions(verimeasure PUBLIC VERIMEASURE_WITH_ARENA)
  target_include_directories(verimeasure PUBLIC
    ${ARENA_INCLUDE_DIR} ${GENICAM_INCLUDE_DIR})
  if(SAVE_INCLUDE_DIR)
    target_include_directories(verimeasure PUBLIC ${SAVE_INCLUDE_DIR})
  endif()
  target_link_libraries(verimeasure PUBLIC ${ARENA_LIBRARY})

  # Both Arena and GenICam are split across several library directories, and on
  # Windows the SDK headers auto-link their import libraries by name through
  # #pragma comment(lib, ...), producing requests for files like
  # GCBase_MD_VC140_v3_3_LUCID.lib. The names come from the headers; all we have
  # to supply is somewhere to find them. Collecting the directories rather than
  # hard-coding a layout keeps this working across SDK versions, where the
  # version and toolset suffixes change.
  set(_arena_lib_glob "")
  if(WIN32)
    file(GLOB_RECURSE _arena_lib_glob
      "${ARENA_SDK_ROOT}/lib64/*.lib"
      "${ARENA_SDK_ROOT}/lib/*.lib"
      "${ARENA_SDK_ROOT}/GenICam/*.lib")
  else()
    file(GLOB_RECURSE _arena_lib_glob
      "${ARENA_SDK_ROOT}/lib64/*.so*"
      "${ARENA_SDK_ROOT}/lib/*.so*"
      "${ARENA_SDK_ROOT}/GenICam/*.so*")
  endif()

  set(_arena_lib_dirs "")
  foreach(_lib IN LISTS _arena_lib_glob)
    get_filename_component(_dir "${_lib}" DIRECTORY)
    # Skip 32-bit directories so the linker cannot pick the wrong architecture.
    # Note the 64-bit GenICam folder is called Win64_i86, so filter on Win32
    # and nothing else.
    if(NOT _dir MATCHES "Win32")
      list(APPEND _arena_lib_dirs "${_dir}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _arena_lib_dirs)

  if(NOT _arena_lib_dirs)
    message(FATAL_ERROR
      "No Arena or GenICam import libraries were found under\n"
      "  ${ARENA_SDK_ROOT}\n"
      "Expected them beneath lib64/ and GenICam/. Check that ARENA_SDK_ROOT "
      "points at the SDK root, the folder containing include/, lib64/ and "
      "GenICam/.")
  endif()

  target_link_directories(verimeasure PUBLIC ${_arena_lib_dirs})
  message(STATUS "Arena library search paths:")
  foreach(_dir IN LISTS _arena_lib_dirs)
    message(STATUS "  ${_dir}")
  endforeach()

  # On Linux nothing names the GenICam libraries for the linker. The Windows
  # headers do it through #pragma comment(lib, ...), which GCC and Clang do not
  # have, and camera.cpp calls into GenICam directly (gcstring, the node
  # pointers, GenericException), so without them the link fails on undefined
  # references. This is the set Lucid's own Linux examples link. Taken by
  # pattern, because the compiler and version suffixes (_gcc54_v3_3_LUCID and
  # the like) change between SDK releases. Linked by full path, so CMake also
  # puts their directories on the build RPATH: the executables start without
  # LD_LIBRARY_PATH, and the linker can resolve what libarena itself needs.
  if(NOT WIN32)
    file(GLOB_RECURSE _genicam_candidates "${ARENA_SDK_ROOT}/GenICam/*.so")
    set(_genicam_libs "")
    foreach(_lib IN LISTS _genicam_candidates)
      # 64-bit only: the SDK ships 32-bit builds alongside in some releases.
      if(NOT _lib MATCHES "Linux32")
        list(APPEND _genicam_libs "${_lib}")
      endif()
    endforeach()
    if(NOT _genicam_libs)
      message(FATAL_ERROR
        "No GenICam libraries (lib*.so) were found under ${ARENA_SDK_ROOT}/GenICam.\n"
        "Check that ARENA_SDK_ROOT points at the SDK root, the folder containing "
        "include/, lib64/ and GenICam/.")
    endif()
    target_link_libraries(verimeasure PUBLIC ${_genicam_libs})
    message(STATUS "GenICam libraries linked:")
    foreach(_lib IN LISTS _genicam_libs)
      message(STATUS "  ${_lib}")
    endforeach()
  endif()

  # Where the matching runtime libraries live, so the message can tell the
  # operator what needs to be reachable at launch.
  if(WIN32)
    set(_arena_bin_dirs "")
    file(GLOB_RECURSE _arena_dlls
      "${ARENA_SDK_ROOT}/*Arena*.dll" "${ARENA_SDK_ROOT}/*GCBase*.dll")
    foreach(_dll IN LISTS _arena_dlls)
      get_filename_component(_dir "${_dll}" DIRECTORY)
      if(NOT _dir MATCHES "Win32")
        list(APPEND _arena_bin_dirs "${_dir}")
      endif()
    endforeach()
    list(REMOVE_DUPLICATES _arena_bin_dirs)
    if(_arena_bin_dirs)
      message(STATUS "Arena runtime DLLs (these must be on PATH at launch):")
      foreach(_dir IN LISTS _arena_bin_dirs)
        message(STATUS "  ${_dir}")
      endforeach()
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------

add_executable(verimeasure_cli apps/verimeasure_cli.cpp)
target_link_libraries(verimeasure_cli PRIVATE verimeasure)

add_executable(verimeasure_station apps/verimeasure_station.cpp)
target_link_libraries(verimeasure_station PRIVATE verimeasure)

# OpenCV_LIBS already carries highgui, so the windowed tools need no extra link.
if(VERIMEASURE_HAVE_HIGHGUI)
  add_executable(phase1_camera_acquire apps/phase1_camera_acquire.cpp)
  target_link_libraries(phase1_camera_acquire PRIVATE verimeasure)

  add_executable(phase2_roi_presence apps/phase2_roi_presence.cpp)
  target_link_libraries(phase2_roi_presence PRIVATE verimeasure)

  add_executable(phase3_4_segmentation_measurement
    apps/phase3_4_segmentation_measurement.cpp)
  target_link_libraries(phase3_4_segmentation_measurement PRIVATE verimeasure)
endif()

# Offline harness. Runs the engine over saved stills, which is how the port was
# checked against the Python original and how a rig can be validated without a
# camera attached.
option(VERIMEASURE_BUILD_TESTS "Build the offline measurement harness" ON)
if(VERIMEASURE_BUILD_TESTS)
  add_executable(offline_measure tests/offline_measure.cpp)
  target_link_libraries(offline_measure PRIVATE verimeasure)

  # Step QC harness. Measures a span across a saved outline and scores it
  # against a style with no camera attached, and carries a --selftest that
  # builds folded lays from known numbers and checks the verdicts.
  add_executable(qc_check tests/qc_check.cpp)
  target_link_libraries(qc_check PRIVATE verimeasure)

  # Edge accuracy bench. Renders a rectangle at a known sub-pixel position,
  # puts it through the real segmentation and refinement, and reports how far
  # the reported boundary sits from the one that was drawn. This is how a
  # change to the edge pipeline is shown to be an improvement rather than
  # asserted to be one.
  add_executable(edge_accuracy tests/edge_accuracy.cpp)
  target_link_libraries(edge_accuracy PRIVATE verimeasure)

  # Traced-curve harness. Renders a seam of known length, points at it as
  # sloppily as a hand does, and checks that the answer does not move. That is
  # the whole claim this feature rests on, so it is the thing the selftest
  # tests: the pointing is allowed to be bad, the reading is not.
  add_executable(curve_check tests/curve_check.cpp)
  target_link_libraries(curve_check PRIVATE verimeasure)

  # Collar detection harness. Renders a collar of known length, draws a box
  # round it a dozen different ways, and checks that the reading does not move
  # - the box is a region, not a position, and that is the claim tested.
  add_executable(collar_check tests/collar_check.cpp)
  target_link_libraries(collar_check PRIVATE verimeasure)

  # Full perimeter harness. Renders a shirt of known perimeter, lays it in two
  # halves the way the station asks, with the clips moved along the edge and
  # the shirt moved about the view, and checks the total does not move - the
  # clips are the join, and where the shirt lies is not supposed to matter.
  add_executable(perimeter_check tests/perimeter_check.cpp)
  target_link_libraries(perimeter_check PRIVATE verimeasure)

  # Laying the garment again for the next step. Renders a shirt refolded where
  # it lies and puts every frame through the engine: a new lay has to be
  # measured without the mat being cleared first, and the lay already measured
  # must never be measured again for a step it was not laid for - untouched,
  # under a hand, or with an arm swept across it.
  add_executable(relay_check tests/relay_check.cpp)
  target_link_libraries(relay_check PRIVATE verimeasure)

  # The startup exposure search, against a simulated camera that delivers
  # frames taken before the exposure was changed, as a real one does. It has
  # to land on the same exposure from any start however far behind the camera
  # is, which is what keeps the picture from being bright on one start and
  # dark on the next. Also checks the exposure remembered in bench.json.
  add_executable(exposure_check tests/exposure_check.cpp)
  target_link_libraries(exposure_check PRIVATE verimeasure)

  # Why the boundary is where it is, from the two pictures the station dumps
  # with its "dump" command. Offline and without a camera.
  add_executable(boundary_probe tests/boundary_probe.cpp)
  target_link_libraries(boundary_probe PRIVATE verimeasure)
endif()

if(NOT VERIMEASURE_WITH_ARENA)
  message(WARNING
    "Building WITHOUT the camera backend.\n"
    "\n"
    "verimeasure_station and verimeasure_cli will refuse to start in this build.\n"
    "Only --calibrate-image and offline_measure, which read saved captures,\n"
    "will work.\n"
    "\n"
    "For a working measurement station, reconfigure with:\n"
    "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \\\n"
    "        -DVERIMEASURE_WITH_ARENA=ON \\\n"
    "        -DARENA_SDK_ROOT=<path containing include/Arena/ArenaApi.h>")
else()
  message(STATUS "Camera backend: Arena SDK at ${ARENA_SDK_ROOT}")
endif()

# ---------------------------------------------------------------------------
# Visual Studio convenience
# ---------------------------------------------------------------------------
#
# Two things otherwise stop F5 working on a fresh clone, and neither says so
# clearly when it bites.
#
# OpenCV's runtime DLLs do not sit next to the executables. Without them on the
# debugger's PATH a run dies with a bare 0xC0000135 and no window, which looks
# like a build problem and is not.
#
# And every tool here resolves templates/, calibration.json and roi_config.json
# against the working directory. The generator points that at the build tree, so
# a debug run reports it cannot open a file that is plainly there.
#
# Both are set per target rather than globally, so nothing outside the Visual
# Studio generator changes.
if(MSVC)
  set(_vm_ocv_root "${OpenCV_INSTALL_PATH}")
  if(NOT _vm_ocv_root)
    set(_vm_ocv_root "${OpenCV_DIR}")
  endif()

  # Any opencv_world or opencv_core DLL will do: all that is wanted is the
  # directory they live in, and the loader picks the right build by name.
  file(GLOB_RECURSE _vm_ocv_dlls
    "${_vm_ocv_root}/opencv_world*.dll"
    "${_vm_ocv_root}/opencv_core*.dll")

  set(_vm_debug_env "")
  if(_vm_ocv_dlls)
    list(GET _vm_ocv_dlls 0 _vm_ocv_first)
    get_filename_component(_vm_ocv_bin "${_vm_ocv_first}" DIRECTORY)
    set(_vm_debug_env "PATH=${_vm_ocv_bin};%PATH%")
    message(STATUS "Visual Studio debug runs will find OpenCV at ${_vm_ocv_bin}")
  else()
    message(WARNING
      "No OpenCV runtime DLL was found under ${_vm_ocv_root}.\n"
      "Debugging from Visual Studio will fail with 0xC0000135 until the folder "
      "holding opencv_world*.dll is on PATH.")
  endif()

  foreach(_vm_target verimeasure_cli verimeasure_station qc_check offline_measure
                     edge_accuracy
                     phase1_camera_acquire phase2_roi_presence
                     phase3_4_segmentation_measurement)
    if(TARGET ${_vm_target})
      set_target_properties(${_vm_target} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
      if(_vm_debug_env)
        set_target_properties(${_vm_target} PROPERTIES
          VS_DEBUGGER_ENVIRONMENT "${_vm_debug_env}")
      endif()
    endif()
  endforeach()

  # A prebuilt OpenCV is almost always compiled with an older toolset than the
  # Visual Studio doing the building. Release builds survive that: MSVC keeps
  # the release runtime binary compatible across versions. The DEBUG runtime is
  # not, so a Debug build against a prebuilt OpenCV dies on startup with
  # 0xC0000139, entry point not found, which looks like nothing at all from
  # inside the IDE. RelWithDebInfo is the configuration to debug in.
  message(STATUS
    "Visual Studio: build Release or RelWithDebInfo. A Debug build only works "
    "against an OpenCV compiled with this same toolset; a prebuilt one will "
    "fail at startup with 0xC0000139.")

  # Without the camera backend this is the only thing that runs end to end, so
  # it is what F5 should start, already pointed at its own checks.
  if(TARGET qc_check AND NOT VERIMEASURE_WITH_ARENA)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                 PROPERTY VS_STARTUP_PROJECT qc_check)
    set_target_properties(qc_check PROPERTIES
      VS_DEBUGGER_COMMAND_ARGUMENTS "--selftest")
  endif()
endif()

install(TARGETS verimeasure_cli verimeasure_station RUNTIME DESTINATION bin)
