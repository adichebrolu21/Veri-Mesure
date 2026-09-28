// Configuration. Everything tunable lives here so the frontend can override it
// without touching logic. Field names, defaults and comments correspond
// one-for-one with the CFG dict in the Python original.

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace verimeasure {

// The lens's radial distortion as measured once for the bench (see
// settleDistortion), with everything the plane model needs to reproduce it:
// the two terms, the centre they act about, the radius they are normalised
// by, and the size of the picture they were fitted on.
struct LensDistortion {
    double k1 = 0.0;
    double k2 = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double norm = 1.0;
    int width = 0;
    int height = 0;
    int frames = 0;
    std::string fitted_at;
};

struct Config {
    // --- Files ---
    std::string calibration_file = "calibration.json";
    std::string roi_config_file = "roi_config.json";
    // What is known about this bench's hardware that no picture of the mat can
    // tell the calibration, and that a recalibration therefore must not throw
    // away: the lens (settleLens), and the exposure the camera runs at, kept
    // so the picture is the same on every start (BenchExposure).
    std::string bench_file = "bench.json";
    std::string results_file;  // empty means no JSON Lines output

    // The style the bench is currently checking. Empty means the station has
    // no step list to work through, so it measures and reports but judges
    // nothing. Set it to run one style per shift; the operator can also switch
    // styles from the page.
    std::string template_file;
    // Where the operator page looks for the templates it offers.
    std::string templates_dir = "templates";

    // --- Physical reference ---
    double grid_pitch_mm = 10.0;  // nominal pitch of the finest printed grid
    std::vector<int> allowed_pitch_multiples{1, 2, 4, 5, 10};

    // --- Grid detection ---
    int min_pitch_px = 20;  // search window for the printed pitch
    int max_pitch_px = 400;
    double line_kernel_pitches = 2.0;      // opening kernel length, in pitches
    double min_line_length_pitches = 6.0;  // reject short segments and arc fragments
    double max_line_thickness_ratio = 0.20;
    int border_margin_px = 24;
    int min_lattice_points = 60;
    double max_index_residual = 0.22;  // fraction of a pitch

    // --- Model fitting ---
    double holdout_fraction = 0.25;
    double distortion_gain_required = 0.02;  // keep k1,k2 only if holdout RMSE drops 2%
    // The lens's distortion, fitted once across several pictures of the rigid
    // mat (verimeasure_cli --fit-distortion) and kept in bench_file.
    //
    // Fitted afresh on every recalibration, the two radial terms are pinned
    // only by whichever lattice points near the edges that one picture
    // happened to index, and two recalibrations of the unmoved rig minutes
    // apart measured the same pixels up to 4.2 mm apart at the edges of the
    // frame and 7.4 mm in the corners, against 0.35 mm in the middle. The lens
    // does not change between those two pictures, so with this set a
    // recalibration fits the homography alone. Unset, it fits both exactly as
    // it always has. See settleDistortion.
    std::optional<LensDistortion> lens_distortion;
    double max_acceptable_rmse_mm = 0.35;
    // The same bar for a grid printed dark on cloth, as far as refusing a
    // calibration goes. Creases move a cloth's grid by a millimetre or so where
    // they cross it, which no plane can follow, so a sound cloth lattice fits
    // up to about 0.4 mm on this bench. Above max_acceptable_rmse_mm it still
    // gets the usual warning to flatten the surface.
    double dark_grid_max_rmse_mm = 0.5;
    // A grid printed dark on a light ground is this bench's working base, the
    // blue cloth, and not a ruler: clamped and creased, its squares measured
    // 1.25% long across the middle and up to 2.8% toward the edges against the
    // rigid mat's. So recalibrating on it retakes the exposure, the background
    // and the working area and keeps the measuring geometry the station already
    // has. Set this to fit the geometry from such a grid anyway. With no
    // geometry to keep, it always is.
    bool dark_grid_sets_geometry = false;
    int lm_iterations = 80;

    // --- A clear base ---
    //
    // Recalibrating takes the background from whatever is on the base, so the
    // base has to be clear. Judged by how much of the working area hides the
    // printed grid: squares two pitches across whose lines stand out from the
    // local level by less than base_grid_min_contrast. On this bench's mat and
    // cloth, lamp on and lamp off, not one square in ~900 did; with a garment
    // down a fifth to three quarters of them do.
    double base_grid_min_contrast = 0.03;
    double base_hidden_max_fraction = 0.01;
    // How long Recalibrate waits for the base to be cleared before giving up.
    double recalibrate_clear_wait_s = 60.0;
    // A geometry kept through a recalibration is checked against the base's
    // printed pitch each time. A change larger than this, in per cent, is what
    // raising or lowering the camera looks like, and is warned about.
    double base_pitch_change_warn_pct = 0.5;
    // Check scale: the thickness of the reference laid on the base, a steel
    // rule by default. It is the reference's top face the camera sees, so it is
    // read at that height rather than at the step's fold height.
    double scale_reference_thickness_mm = 1.0;
    // Check scale read in more than one place - the middle, and toward the
    // sides - corrects the scale only when every place asks for the same
    // correction to within this share. Places that disagree are not a scale
    // error but the lens or the corners, and no one factor mends that, so
    // nothing is applied and the numbers are shown instead.
    double scale_check_agree = 0.003;

    // --- Camera ---
    // Set the exposure from the picture when there is none saved for this bench
    // (bench_file), and again on Recalibrate. Off, the camera keeps whatever it
    // was left at.
    bool auto_expose_on_start = true;
    double target_highlight_level = 225.0;  // 99.5th percentile target, 0-255
    // How much of that to actually aim for when the thing being metered is an
    // empty mat.
    //
    // Auto-exposure runs once at startup on a clear surface, and a clear
    // surface is the darkest scene this bench will ever see. Filling the range
    // with it leaves nothing for the garment: measured here, a mat exposed to
    // the top of its range was followed by a salmon shirt with 48% of its red
    // channel pinned at 255 - and clipped cloth has no texture to recognise, no
    // edge profile to fit, and it blooms into the mat beside it.
    //
    // 0.62 puts the empty mat's brightest channel near 140 and leaves a garment
    // about 1.8x that before anything saturates, which covers pale cloth on a
    // dark mat. Lower it if the WARNING about clipped white still appears.
    double clear_mat_headroom = 0.62;
    // The level the clear base is metered to, when this bench has been given
    // one (--exposure-target, kept in bench_file; see settleExposureTarget).
    // Unset, it is target_highlight_level x clear_mat_headroom, about 140.
    //
    // A dark garment sits about 20 levels up at 140, where one level of noise
    // is a large share of what it has, so a bench that measures dark cloth
    // may want the picture brighter. Every level added comes off the headroom
    // a pale garment has before it clips white, and clipped cloth blooms out
    // over the mat and reads long.
    std::optional<double> exposure_target_level;
    double auto_expose_tolerance = 8.0;
    int auto_expose_max_steps = 12;
    // How long to wait after a change of exposure for the picture to show it
    // and hold still, in frames, and how many frames the whole search may use.
    // The first frames after a change are usually still ones taken before it;
    // see meterExposure. Two readings within this share of each other (or two
    // levels) are the same picture.
    int auto_expose_settle_frames = 8;
    int auto_expose_max_frames = 30;
    double auto_expose_agree_fraction = 0.03;
    double pixel_size_mm = 0.0024;  // TRI200S sensor pitch, 2.4 um
    // The lens, which is what the working distance for the parallax correction
    // is worked out from. Remembered in bench_file between runs; see settleLens.
    std::optional<double> lens_focal_length_mm;

    // --- Throughput and liveness ---
    // Detection runs on a decimated copy. Full 20 MP segmentation took 7.5 s per
    // frame and allocated over a gigabyte of temporaries, which is the whole
    // reason the feed felt frozen. Nothing is lost: this scale only decides
    // whether an object is present and roughly where. The measurement itself
    // still runs at full resolution on a crop.
    int detect_step = 6;
    int stream_buffers = 8;
    int grab_timeout_ms = 4000;
    int max_consecutive_grab_failures = 12;
    int crop_pad_px = 48;  // margin around the object for the fine pass
    // Gap bridging, as a fraction of the detected grid pitch. Sized to close a
    // printed line, never wide enough to join two parts sitting side by side.
    double mask_bridge_pitch_fraction = 0.30;
    int mask_bridge_px_fallback = 15;
    // How thin a protrusion has to be before it counts as a whisker rather than
    // part of the garment, as a fraction of the bridge above - so it is a
    // physical width, about a millimetre and a half on this bench, and it stays
    // one on the decimated pass.
    //
    // Whiskers are what the boundary grows when the printed lines beside a
    // clipping garment light up: a few pixels wide, reaching out onto the mat,
    // walked down one side and back up the other so the drawn outline sprouts
    // teardrops. They are removed by reconstruction rather than by an opening,
    // which is what lets them go without every corner of the garment being
    // rounded by the same radius. Set to 0 to leave the boundary exactly as the
    // difference found it.
    double outline_filament_fraction = 0.55;

    // Follow the base where it has moved since the background was taken; see
    // creep.hpp. A cloth base is dragged by the garment laid on it, and every
    // printed line it moves is a difference the fine pass would take for
    // garment. The movement is read on the bare base round the lay and the
    // background resampled to match. Unless base_creep_min_patches places on
    // the base moved by base_creep_min_px or more, nothing is resampled and the
    // fine pass runs exactly as it did without this.
    bool base_creep_align = true;
    double base_creep_min_px = 0.5;
    // How far out from the crop, and so from the garment, the base is read.
    int base_creep_band_px = 192;
    // The rim round the decimated outline that is not taken for base: that
    // outline is a cell or so out, and the garment's shadow lies there.
    int base_creep_margin_px = 24;
    // A patch this much garment or rim is not read.
    double base_creep_max_cover = 0.15;
    // How far the movement is smoothed. On this bench it changed by 7 px over
    // 600 px of cloth beside one sleeve.
    double base_creep_smooth_px = 200.0;
    // A correlation weaker than this has not found the printed pattern.
    double base_creep_min_response = 0.3;
    // A patch this far from what the base round it says has read something
    // other than the base.
    double base_creep_outlier_px = 2.0;
    // Fewer patches than this read, or fewer than this that moved, and the
    // base is taken not to have moved.
    int base_creep_min_patches = 6;

    // --- Liveness watchdogs ---
    int present_frames_required = 2;
    int clear_frames_required = 3;
    double stabilise_timeout_s = 40.0;
    double measured_timeout_s = 900.0;
    double background_stale_fraction = 0.60;
    double background_stale_seconds = 10.0;
    bool illumination_normalise = true;
    // Where that gain is read from once the garment is a large part of the
    // view.
    //
    // The gain is a median over the working area of how much brighter each
    // pixel is than the background, and a median measures the light only
    // while most of what it is taken over is mat. Past half the view it
    // measures the garment: on a real flat lay a pale shirt took it from 1.04
    // at 45% of the view to its 2.0 clamp at 58%, cutting folds and shaded
    // cloth out of the outline, and a dark shirt takes it to its 0.5 clamp
    // just past half, where the mat reads as the object and the outline
    // becomes the frame. So when the median comes out below the trust band
    // below, the gain is read again from the mat alone - the samples outside
    // the garment's filled outline, grown by gain_mat_margin_px - and the
    // decimated pass is run again with it.
    //
    // Only for a lay darker than the base, unless gain_mat_only_brighter is
    // set. A bright lay is the other way round on the blue cloth: the shirt
    // lights the cloth beside it, far past any margin, and a gain read from
    // the mat takes that glow for cloth. On a real white shirt covering 53%
    // of the view the outline ran up to 6.4 mm (p90) outside the shirt at the
    // mat's gain of 1.05, against 0.2 mm inside at the whole-view gain of 2 -
    // which cuts shaded cloth on a dark mat, but on this base hides the glow.
    double gain_mat_only_coverage = 0.5;
    // A whole-view gain outside this band is not believed without the mat
    // agreeing. A broken median leaves it before the coverage says so: the
    // first pass under-reads how much is covered once the gain is wrong.
    double gain_trust_band_low = 0.8;
    double gain_trust_band_high = 1.25;
    // Also read the gain from the mat for a lay brighter than the base: at
    // gain_mat_only_coverage of the view, or above the band's top. Off; see
    // above.
    bool gain_mat_only_brighter = false;
    // How far past the garment's outline the mat is not asked, in
    // full-resolution pixels. The shadow and the glow beside a garment are
    // the mat under different light, not the light.
    int gain_mat_margin_px = 48;
    // The least share of the samples that must still be mat for the mat to be
    // believed. Below it the gain last measured with nothing on the mat is
    // used - 1 after a new background - and never the whole-view median,
    // which is what failed.
    double gain_mat_min_fraction = 0.05;
    double chroma_diff_threshold = 0.025;
    // Settling is judged on contour area and centroid, not on a bounding box.
    // A minimum-area rectangle has to enclose every point, so on a decimated
    // contour it jitters by more than a millimetre frame to frame and a tight
    // tolerance can never be met. Area and centroid are integral statistics and
    // they are what "the part has stopped moving" actually means.
    double stability_area_fraction = 0.006;
    double stability_centroid_px = 2.0;

    // --- Laying the garment again for the next step ---
    // How a new lay is told from the one the held reading was taken from, once
    // the station has asked for one (MeasurementEngine::awaitNewLay). Both are
    // compared as the decimated pass sees them, filled, so a print or a crease
    // inside the cloth is not a change.
    //
    // Boundaries closer together than this are the same boundary. The
    // decimated outline of a lay nobody has touched moves by a cell now and
    // then, and that is not the garment moving.
    double relay_tolerance_mm = 2.0;
    // Cloth gone from where the held lay was. A hand can put something into
    // the picture but it can never uncover the mat under the garment, so this
    // can be small: a twentieth of the lay, and never under 2000 mm2, about
    // 45 mm square. Refolding for another step takes away far more than that.
    double relay_removed_fraction = 0.05;
    double relay_removed_min_mm2 = 2000.0;
    // New cloth beside the held lay with none taken away: a fold opened out
    // where it lies. An arm reaching across the mat adds area too, and an arm
    // is how a seam is pointed at on the lay being held, so this has to be
    // more than an arm can add.
    double relay_added_fraction = 0.30;
    double relay_added_min_mm2 = 60000.0;
    // Retaking the reading being held. That lay was laid for the step being
    // retaken, so straightening the fold where it lies is the retake, and a
    // change this size in either direction is enough.
    double relay_retake_fraction = 0.01;
    double relay_retake_min_mm2 = 800.0;
    // Consecutive frames the change has to be seen on before the held reading
    // is let go, so one frame of a hand sweeping across does not do it.
    int relay_confirm_frames = 2;

    // --- Background ---
    int background_average_frames = 10;
    // How many consecutive frames have to match before the surface counts as
    // still.
    //
    // The floor is background_average_frames, and it is worth saying why. The
    // frames that go into the background reference are the last ones of this
    // run, so at this setting every single frame that is averaged has been
    // checked against its neighbour and found unchanged, which is the property
    // that actually matters. Anything above the floor is pre-roll: frames spent
    // proving the surface was still BEFORE the ones that get used, each costing
    // half a second of startup at this camera's frame rate. It was 12 when the
    // background was captured in a second pass afterwards, where the whole run
    // was pre-roll by construction. Raise it if this bench sits somewhere that
    // shakes and you want stillness demonstrated for longer before it counts.
    int scene_settle_frames = 10;
    double scene_settle_tolerance = 1.2;  // mean abs frame-to-frame delta, gray levels
    // How large a patch the stillness test averages over before comparing two
    // frames.
    //
    // Read noise is per pixel and independent between frames, so comparing
    // single pixels puts a floor under the measured delta of about 1.1 sigma no
    // matter how still the bench is. On this camera, at the exposures a lit
    // bench actually needs, that floor sits above the tolerance: the surface
    // was never once declared static, every startup and every recalibration
    // paid the full timeout instead, and the reading told you about the sensor
    // rather than about the mat. Averaging each block of pixels first divides
    // that noise by the width of the block while leaving real movement, which
    // covers far more than one block, essentially untouched. The question and
    // the tolerance are unchanged; only the noise in the answer is gone.
    int scene_settle_block_px = 8;
    double background_drift_blend = 0.02;  // slow EMA while the surface is idle
    double background_drift_idle_seconds = 4.0;

    // --- Segmentation ---
    double chroma_angle_threshold = 0.020;  // 1 - cos(angle) between BGR vectors
    double intensity_ratio_high = 1.25;     // much brighter than background
    double intensity_ratio_low = 0.35;      // much darker, i.e. not a shadow
    double shadow_uniformity = 0.10;        // channel ratio spread below this is shadow
    // How dark a uniformly-darker patch may go and still be read as a shadow
    // rather than as an object.
    //
    // Under a lamp rather than a window a cast shadow has no fill light in it
    // and goes far darker than a diffusely lit room ever produced, so a floor
    // set for window light classes the shadow beside a folded garment as part
    // of the garment and every measurement across it reads long. Below 0.25 the
    // colour direction has to agree that it is a shadow as well, which is what
    // keeps a genuinely dark object from disappearing into this. Raise it
    // towards 0.25 to restore exactly the old behaviour.
    double shadow_ratio_floor = 0.06;
    // What a shallow shadow - uniformly darker, above a quarter of the
    // background - needs besides its colour before it is taken for the
    // surface.
    //
    // It is the one way into "this is the surface" that asks for no evidence
    // at all, and a dark garment of the base's own colour, or a neutral one
    // on a neutral base, satisfies it outright at 0.25 to 0.35 of the
    // background, where it would otherwise be foreground for being too dark.
    // 0 asks for nothing more, as always. 1 also asks for the base's printed
    // pattern to show through in the same places (the evidence the other
    // shadow tests use). 2 asks for the colour not to have moved by more than
    // half of chroma_angle_threshold. Each costs a pale garment something
    // where the evidence is silent - between two printed lines, and in the
    // part of a shadow the texture window cannot see - so it is off.
    int shadow_shallow_needs_evidence = 0;
    // Telling a deep shadow from something black by whether the mat's printed
    // grid is still showing through it. How wide a neighbourhood to judge that
    // over, and how much of the background's texture a shadow has to have kept,
    // as a fraction of what falling that dark would have left of it. The window
    // has to FIT INSIDE the shadow being judged, which is the binding
    // constraint and the counter-intuitive one. A shadow beside a folded
    // garment is 25 to 60 px wide; a window wider than that straddles the
    // garment and the clear mat either side, and reports a correlation
    // belonging to neither - 0.04 for a 97 px window inside a 40 px shadow,
    // against 0.65 for a 21 px one. Narrower sees less of the mat to judge by,
    // but what it does say it says reliably: across every window from 9 to
    // 41 px, the fraction of a pale garment mistaken for shadow is 0.0%. The
    // gaps are filled by shadow_region_fraction, not by widening this.
    // Small, because it has to fit inside the narrowest shadow worth catching.
    // A 25 px shadow has nowhere to put a 21 px window that is not also
    // touching the garment, so at 21 px a narrow shadow is never seeded at all.
    int shadow_texture_window_px = 13;
    double shadow_texture_keep = 0.45;
    // How closely the surface seen now has to correlate with the surface as it
    // was, over that window, before a darker patch counts as a shadow.
    //
    // This is the only shadow test here that says nothing about colour, and it
    // is the one that matters most under a key light. A shadow is not lit by
    // the light casting it, it is lit by whatever else is in the room, so it
    // arrives a different colour as well as darker - measured on this mat, a
    // colour angle of 0.035 against a 0.020 threshold and a channel spread of
    // 0.245 against 0.10. Both colour tests call that an object, the outline is
    // drawn around the outside of it, and the reading comes back 5 to 9 mm long
    // depending on how wide the shadow is. A correlation is immune to all of
    // that: the printed grid either still shows through in the same places or
    // it does not. Lower it if shadows are still being measured; raise it
    // towards 0.8 if something opaque is being mistaken for one.
    double shadow_texture_correlation = 0.5;
    // How far the verdict spreads. This must exceed the gap between two
    // printed grid lines - at a 72 px pitch, comfortably more than 72 - or a
    // pixel in the middle of a gap hears from nobody and the shadow survives in
    // stripes, which holds the outline out just as far as surviving everywhere
    // would. It can afford to be this wide because the vote is taken over the
    // darker pixels in the window rather than over all of them, so a garment
    // lying across most of it does not dilute the answer.
    int shadow_region_window_px = 121;
    // The same idea for a glow, at a fraction of the reach. A shadow can be
    // spoken for from far away because the garment is not in the population
    // being polled - cloth is not darker than the mat it lies on. A glow can
    // not: a pale garment is the brightest thing in the picture, so it sits in
    // the very population the vote counts, and a far-reaching one carried by
    // the bounce beside a sleeve took 33 mm off a 225 mm reading. Short enough
    // that the worst it can cost is half its width, and only where the garment
    // is itself clipped.
    int glow_region_window_px = 25;
    // Whether the bench tries to recognise a glow. Off, and it should stay off
    // until something here changes.
    //
    // The glow is real and worth removing: a pale garment under a bright lamp
    // throws light back onto the mat beside it, measured on this bench at twice
    // the brightness of the same mat a hand away, and the outline runs 7 mm out
    // into it. Everything needed to recognise it is already here - a glow is
    // the same thing a shadow is, this surface under different light, and the
    // same correlation settles both.
    //
    // What is not solved is that a glow shares its population with the cloth.
    // A shadow can be recognised by a vote among neighbouring darker pixels
    // because a garment is not darker than the mat it lies on, so the vote can
    // never reach it. A garment IS brighter, so every mechanism that recognises
    // a glow can also eat the garment - and it does: switched on, this took
    // 33 mm off a 225 mm sleeve on the bench, and it costs 58% on the edge
    // accuracy selftest, 0.87 px of point RMS against 1.38 px.
    //
    // That failure runs the wrong way. Everything else that goes wrong in this
    // file adds something to the outline, and a reading that is too long is
    // obvious and bounded. A reading that is too short is indistinguishable
    // from a garment that is out of spec, which is the one answer this bench
    // must never invent. So the glow stays measured as part of the garment,
    // which reads a few millimetres long, until it can be removed without
    // putting the cloth at risk.
    bool segment_handles_glow = false;
    // What fraction of a neighbourhood has to have been called shadow on
    // texture evidence before a darker pixel with no evidence of its own joins
    // them. Between two printed grid lines there is nothing to judge by, and
    // without this those gaps stay foreground and hold the outline out at the
    // full width of the shadow however well the lines themselves were read.
    // An eighth, because this evidence is sparse by nature and very nearly
    // never wrong: at its best it speaks for a little over half a shadow -
    // between two printed lines there is nothing to speak about - and across
    // every window size tried it mistook 0.0% of a garment for one. A threshold
    // set as though the evidence were dense throws away the only thing it is
    // good for.
    //
    // Measured by sweeping it: at 0.20 the innermost few pixels of a narrow
    // shadow still survive, because there the evidence window is overlapping
    // the garment and cannot vouch for anything. At 0.12 every shadow tried
    // clears completely, and a navy garment lying beside one - the case that a
    // loose threshold would eat into, since it is genuinely darker than the mat
    // and genuinely has shadow next to it - still measures +0.07 mm. Below 0.10
    // that garment starts to be eroded.
    double shadow_region_fraction = 0.12;
    // A garment darker than the base keeps what the vote above would take from
    // it.
    //
    // The vote is safe only while cloth is never darker than the base, and on
    // the blue cloth a black shirt reads a seventh of it: the shadow beside
    // the shirt carried the vote 5 to 7 mm into every edge and deep into the
    // corners, and on the decimated pass, where the same window reaches six
    // times as far, it cut the region the fine pass is allowed to look at. So
    // when the median brightness under the lay's outline is below
    // dark_garment_ratio of the base's, the vote may not take a pixel where the
    // base has a pattern to show and does not show it. A shadow shows it and
    // still goes. A lay as bright as the base or brighter - every white shirt
    // - never takes this path.
    bool dark_garment_keeps_evidence = true;
    double dark_garment_ratio = 0.8;
    // A garment near black is told from its own shadow by how dark it is.
    //
    // The pattern test above has a blind side, and a black shirt found it.
    // Where the shirt's shadow falls across a printed line, or the base has
    // crept a pixel since it was taken, the line is darker and no longer quite
    // where it was, which is exactly what the test takes for cloth: the
    // outline ran out along those lines 3 to 4 mm onto the base. And between
    // two lines, where the base has no pattern to test, the vote still took
    // the cloth a grid cell at a time, up to 14 mm deep.
    //
    // Black cloth needs no pattern to be told from its shadow. On this bench a
    // black shirt reads a tenth of the base under it and the shadow beside it
    // half of the base or more. So when the lay's middle brightness is below
    // dark_garment_black_share of the base's, a pixel as dark as the cloth -
    // at most dark_garment_black_margin times the cloth's middle brightness,
    // and never more than dark_garment_black_cap of the base's - is cloth and
    // nothing takes it, and a pixel brighter than that has no witness to keep
    // it from the vote. A garment any lighter keeps the rule above, and 0
    // turns this off.
    double dark_garment_black_share = 0.15;
    double dark_garment_black_margin = 1.8;
    double dark_garment_black_cap = 0.2;
    int morph_kernel_size = 5;
    double min_contour_area_mm2 = 100.0;
    // An outline that runs along three or four sides of the working area is
    // the edge of the picture, not a garment.
    //
    // It is what the base looks like once it has been taken for the object. On
    // 2026-09-24 a shirt covering more than half the view drove the gain to its
    // 0.5 clamp, the mat went into the mask, the outline became the frame and a
    // "hem" of 598.5 mm was read off the bottom of the picture. No lay does
    // that - a half of the perimeter step touches one side, a hem running off
    // the picture two - so such an outline is refused with a reason of its own
    // and never measured. A side is touched when the outline's bounding box
    // comes within outline_edge_touch_px of it, and it takes
    // outline_refuse_sides of the four. A garment wider and taller than the
    // view touches three as well - a fold at the rail with the rest of the
    // shirt running off the top and both sides - so raise it to 4 if lays
    // like that are measured on this bench.
    bool refuse_frame_outline = true;
    int outline_edge_touch_px = 3;
    int outline_refuse_sides = 3;

    // --- Edge refinement ---
    bool subpixel_enabled = true;
    double subpixel_search_px = 7.0;
    double subpixel_step_px = 0.25;
    double subpixel_blur_sigma = 1.6;
    double contour_resample_px = 3.0;
    int contour_smooth_window = 5;

    // Scharr rather than Sobel for the gradient the refinement locks onto.
    // Same cost, same shape, but its weights are the ones that actually make
    // the operator rotationally symmetric, so a diagonal edge is located with
    // the same accuracy as an axis-aligned one instead of slightly worse.
    bool subpixel_scharr = true;

    // A fabric edge is continuous. A boundary point whose correction disagrees
    // with its neighbours by more than this has not found a better reading of
    // the edge, it has found something else, and it is pulled back to what the
    // neighbours agree on. Set to zero to keep every correction as found.
    double subpixel_shift_outlier_px = 1.25;
    int subpixel_shift_median_window = 7;  // boundary samples, forced odd

    // --- Stability and averaging ---
    int stable_frames_required = 4;
    double stability_tolerance_mm = 0.25;
    int measurement_average_frames = 3;

    // --- The folded edge ---
    // Which side of the picture the operator presents the fold to. The machine
    // looks in from here and measures the first straight edge it meets, so this
    // must be the same side every time; it is marked on the live feed.
    //
    // The bottom, because the view is wider than it is tall and a shirt is
    // wider than it is tall. A rail down one side leaves only the height of the
    // frame to measure across, which is not enough to take a half chest: the
    // fold runs off the top and bottom of the picture and only part of it can
    // be read. Along the bottom the same fold lies across the long axis of the
    // view and fits, so the whole span is in frame.
    std::string scan_from = "bottom";
    // How deep past the first fabric the edge may lie before it stops counting
    // as the near edge. Wide enough that a fold need not be perfectly square to
    // the camera, narrow enough that a different edge of the garment further in
    // cannot be picked instead.
    double edge_band_mm = 40.0;
    // How rough the edge may be: the RMS of the boundary about the arc fitted
    // through it. This is a measure of the cloth, not of the shape - bunching,
    // a wrinkle riding over the fold, a boundary that will not settle. The
    // sweep of the edge itself is not roughness and is not counted here; see
    // edge_max_bow_fraction. Recorded on every reading whether or not it trips
    // this; the limit is here to catch a fold that has gone badly wrong, not to
    // police ordinary handling.
    double edge_straightness_mm = 3.0;

    // How much of the edge the fit looks at while it follows it.
    //
    // A garment edge is not a straight line and never was. A hem sweeps, a
    // shoulder slopes, a chest line crosses a body that is wider at the bottom
    // than the top; on the shirts this was written for that is 13 to 15 mm of
    // sweep across half a metre. A fit stretched across the whole edge counts
    // that curvature as error and gives up partway along, which read a 540 mm
    // hem as 444 mm and a 430 mm shoulder as 362 mm. A fit that sees only this
    // much at a time finds a curved edge straight everywhere it looks, and
    // follows it the whole way.
    double edge_tangent_window_mm = 40.0;
    // How far the edge may turn, end to end, before it has become another edge.
    //
    // This is what ends a run now, and it is the right question: a hem sweeping
    // 15 mm over half a metre turns about 13 degrees end to end, while an
    // armhole, a side seam, or the corner between two edges turns far more than
    // that within a few millimetres. So the edge ends at the corner, which is
    // also where the seam a tape would be measured to actually is.
    double edge_turn_limit_deg = 30.0;
    // How far past the run the measurement may reach to find the corner.
    //
    // Fabric does not turn a square corner, it rounds one, and a tape is laid
    // to where the two edges would have met rather than to where the rounding
    // began. So the extent carries on past the turn while the boundary is still
    // recognisably this edge, and the corner is taken from the arc. Bounded,
    // because past a point this stops being the edge and starts being the next.
    //
    // A bound on a computed answer, not a distance to walk. The seam is found
    // by carrying this edge and the one leaving it on until they cross; this is
    // only how far that crossing is allowed to be before it is not believed.
    // Set to zero to measure to where the cloth stops being flat instead, which
    // reads short by about twice the radius the corner is rounded through.
    double edge_corner_reach_mm = 20.0;
    // How far apart the two edges have to point before their crossing means
    // anything.
    //
    // This is the guard that tells a seam from a fold. A side seam leaves the
    // hem at something near a right angle and the crossing is well conditioned.
    // A fold does not turn a corner at all - the cloth carries on, or doubles
    // back on itself - so the two directions are nearly parallel, the crossing
    // is anywhere or nowhere, and no corner is invented. That is exactly the
    // case that once added a centimetre the garment did not have.
    double edge_corner_min_angle_deg = 45.0;
    // How much boundary past the end of the edge to look through for the edge
    // that leaves it. Wide enough to see past a generously rounded corner,
    // narrow enough that it is still the same corner.
    double edge_corner_search_mm = 120.0;

    // Whether the measurement follows the edge or crosses it.
    //
    // A tape laid flat between two seams reads the chord, and that is what a
    // tech pack states, so that is the default. Following the edge instead is
    // the honest answer to "how long is this hem", and it is what the operator
    // sees traced on the feed. At the curvatures a garment actually has the two
    // differ by about a millimetre in five hundred, so this is a decision about
    // which definition the floor works to rather than one about accuracy.
    bool edge_measure_along_curve = false;
    // How far the edge may bow, as a share of its own length.
    //
    // The curvature belongs to the garment, so the chord across it is still the
    // measurement. What this catches is a fold gone wrong, where the cloth
    // between the corners is slack and the corners are no longer on the seams.
    // A share rather than a millimetre count, because 14 mm of sweep means one
    // thing across half a metre of chest and another across a 230 mm sleeve.
    double edge_max_bow_fraction = 0.05;
    // How much of the cloth lying along the edge has to read as that one edge.
    // The backstop for a run that stopped early on something the other limits
    // do not describe.
    double edge_coverage_fraction = 0.75;
    // Shorter than this is not a fold, it is a coincidence.
    double edge_min_length_mm = 60.0;

    // How far one boundary point may sit off the line before it stops counting
    // as part of the edge.
    //
    // A garment does not present a clean outline. A thread, a nick in the fold,
    // a care label lying over the edge, a speck of the mat that segmented as
    // cloth: each is a spike of a few points on a boundary of several thousand.
    // A search that ends the edge at the first such point does not measure a
    // fold with a thread over the middle of it, it measures the longer of the
    // two halves the thread cut it into, and reports that as the garment.
    double edge_defect_reject_mm = 7.5;
    // How far the edge may advance while one of those interrupts it.
    //
    // This is what separates a defect from the end of the edge, and it is a
    // question of where the boundary comes back rather than how far it strayed.
    // A defect sits across the edge and gives it back within a few millimetres
    // of where it took it. A corner, or a second panel of cloth, carries on
    // somewhere else. So the run steps over the first and stops at the second,
    // however dramatic the excursion looked. Ten times this in boundary length
    // is also the most any single defect may spend, which bounds the search.
    double edge_defect_bridge_mm = 12.0;

    // How close to the border of the working area an edge may end before it has
    // to be read as running past it.
    //
    // A fold longer than the picture still produces a perfectly straight edge,
    // a perfectly good straightness figure and a number that is simply the
    // width of the view. That is the one failure that looks exactly like a
    // measurement, so it is called out rather than reported.
    double edge_clip_margin_mm = 3.0;

    // --- The traced curve ---
    //
    // Everything above measures the outline of the cloth: a fold presented to
    // the rail, found by differencing against the empty mat. A neckline is not
    // that. It is a seam in the middle of the garment, the neck opening shows
    // the back panel rather than the mat, and the background difference sees
    // nothing there at any resolution. So the operator has to say where it is.
    //
    // What they say is not the measurement. A fingertip sits about ten
    // millimetres above the mat and the plane model is a homography onto the
    // mat, so at the edge of this frame that height alone displaces it four
    // millimetres, and the displacement grows from the centre outward so a
    // straight trace comes back bent. Add the ambiguity about where the tip is
    // under an occluding nail, the shake of a hand, and the fact that a finger
    // dragged along ribbed jersey stretches the thing being measured, and a
    // traced path is not a reading of the garment. It is a reading of the
    // operator.
    //
    // So the trace only ever says WHICH curve and ROUGHLY where. A corridor is
    // built around it and the seam is found inside that corridor from the
    // picture, to a fraction of a pixel, exactly the way every other edge in
    // this pipeline is found. The pointing has to be good to about a
    // centimetre. The answer is good to a fraction of a millimetre. That gap is
    // the whole design, and every default below is chosen to keep it open.

    // How far either side of the traced path the seam is looked for.
    //
    // This is the budget for everything the pointing gets wrong: parallax,
    // where the tip really is, hand shake, and a tablet touch landing four
    // display pixels off.
    //
    // Sized by the middle point rather than the ends, which is the part that
    // was got wrong at first. Pointing at an END of a seam is easy - it is a
    // corner, there is something to aim at, and a few millimetres is what it
    // costs. Pointing at the deepest part of an arc is nothing like as easy,
    // because there is no landmark there at all: the operator is estimating
    // where the middle of a curve ought to be. On the first real jersey that
    // came to this bench the two ends landed on the collar seam and the middle
    // point sat twenty-five millimetres above it, so with a fifteen-millimetre
    // corridor the seam was OUTSIDE the search for the whole middle of its run
    // and there was nothing to find.
    //
    // And the middle point is the one that sets the shape, because the arc goes
    // exactly through all three: an error there bends the whole corridor rather
    // than shifting it. Thirty millimetres covers that with room over. Wider
    // still starts running the corridor off the picture, and admits more of the
    // garment for the prior to have to discriminate against.
    double curve_corridor_mm = 30.0;
    // How finely the corridor is sampled, along it and across it.
    //
    // In millimetres rather than pixels on purpose. The corridor is rectified
    // into a straight ribbon before anything looks at it, so sampling it in
    // millimetres makes the smoothing, the corridor width and the smoothness
    // penalty below all mean the same thing on any lens at any working
    // distance. At this bench's 0.138 mm per pixel a quarter-millimetre step
    // oversamples slightly, which is what sub-pixel work wants.
    double curve_sample_mm = 0.25;
    // How much the picture is smoothed, across the corridor, before the seam is
    // looked for.
    //
    // A seam is not a step in intensity. It is a band a millimetre or two wide
    // with stitching in it, and the stitches are individually sharper than the
    // seam. Smoothing at about the width of the feature stops the search
    // locking onto one stitch and losing the next.
    double curve_blur_mm = 0.6;
    // How much of the seam's own length is averaged before it is looked for.
    //
    // This is the one that separates a seam from cloth, and it works because a
    // seam is the only thing in the corridor that RUNS. Knit texture is a
    // millimetre across and goes nowhere; a crease wanders; a speck is a speck.
    // A seam holds its line for the whole length of the corridor, so averaging
    // three millimetres of it along its own direction lifts it well clear of
    // everything that does not, and costs nothing across, where the accuracy
    // lives.
    //
    // The corridor is rectified before this is applied, so "along" means along
    // the seam and not along a row of the picture. That is most of why the
    // corridor is rectified at all.
    double curve_along_mm = 3.0;
    // How far above the corridor's own noise a feature has to be before it
    // counts as a line at all, in robust standard deviations.
    //
    // Robust, and saturating, and both of those matter. Robust because a
    // topstitch or a fold sets the maximum of any corridor it appears in, so a
    // scale taken from the maximum makes the seam next to it look like nothing.
    // Saturating because ONCE A LINE IS CLEARLY A LINE, BEING BRIGHTER MUST NOT
    // MAKE IT MORE CORRECT. A neckline has four lines within a centimetre - the
    // neck opening, the binding edge, the attachment seam, the topstitch - and
    // the one the tech pack means is rarely the strongest. Which one is wanted
    // is the operator's to say and nothing in the picture knows it, so once
    // every candidate reads as a line, the pointing decides between them.
    double curve_evidence_sigmas = 4.0;
    // ...and how much of the picture it has to change across itself, in gray
    // levels, whatever the noise happens to be.
    //
    // The floor under the scale above, and it is not optional. A scale taken
    // only from the corridor's own noise divides by whatever little noise it
    // finds, so on smooth cloth the TAILS of a feature clear four sigma just as
    // its middle does: the evidence goes flat for a couple of millimetres
    // either side of the seam, the search has nothing to choose between inside
    // that band, and the pointing places the path. Which is exactly the failure
    // this whole file exists to prevent, arriving through the back door.
    //
    // A garment seam changes the picture by five to fifteen gray levels across
    // itself. Smoothed sensor noise on this bench is well under one. Two is a
    // long way from either.
    double curve_min_contrast = 2.0;
    // How far off the pointing a line has to be before the pointing outweighs
    // it.
    //
    // The other half of the same idea, and what actually chooses between four
    // parallel lines. The cost of sitting somewhere grows as the square of how
    // far it is from what the operator pointed at, reaching the value of a
    // perfect line at this distance.
    //
    // Square rather than straight so that it is flat at the bottom: near the
    // middle of the corridor it has no opinion at all, which is what leaves the
    // picture free to place the seam to a fraction of a millimetre. Only
    // further out does it start to bite. Five millimetres puts the neck opening
    // six millimetres in, and the topstitch two out, on the wrong side of a
    // tap that landed within about three - which is what a tap is worth.
    //
    // It cannot move the answer along the seam and it cannot bias the reading:
    // the sub-sample refinement below runs on the picture alone. This chooses
    // WHICH line. The picture still says where that line is.
    double curve_prior_mm = 3.0;
    // How far the seam may wander from the line that was chosen for it.
    //
    // The search runs in two stages. The first picks one line for the whole
    // length, out of a rigid family - a shift, a tilt and a bow off the arc
    // through the taps - so that choosing a line is a decision taken once and
    // paid for over the whole run. The second lets the path follow the cloth
    // freely again, but only this far either side of that line.
    //
    // The separation is the point. A path free to move slowly is free to end up
    // anywhere: it pays for the RATE at which it crosses the corridor and never
    // for how far it has gone, so a sample every twentieth row costs nothing
    // and arrives on a different line four hundred rows later, smooth and
    // confident and wrong. Two millimetres covers every wobble a garment seam
    // actually has and does not reach the next line along.
    double curve_follow_mm = 1.2;
    // What a step sideways has to buy before it is worth taking.
    //
    // The seam is found by walking the corridor end to end and choosing, at
    // every sample along it, how far across to sit. Sitting where the picture
    // says an edge is scores; moving sideways to get there costs this much per
    // sample of movement, in the same units the evidence is measured in, which
    // are normalised so that one is the strongest gradient anywhere in the
    // corridor.
    //
    // It is the only thing standing between a seam and a wrinkle. Too low and
    // the path chases every crease it passes, arriving somewhere plausible and
    // wrong. Too high and the path is a straight line that has stopped reading
    // the picture at all - which is the more dangerous failure, because a
    // straight line through a corridor drawn by the operator looks exactly like
    // a successful snap. Both are caught by drift_mm and by the confidence,
    // which is why both are on every reading.
    double curve_smoothness = 0.35;
    // How much of the seam is looked at when smoothing the snapped path.
    //
    // The path comes back one sample at a time and each sample carries its own
    // noise, a few hundredths of a millimetre of it. That does not matter for
    // the chord, which is two points, and it matters a great deal for the arc,
    // which is eight hundred of them added end to end: noise that averages to
    // nothing still lengthens every single segment, and a fortieth of a
    // millimetre of jitter per sample inflates a 200 mm neckline by four. It
    // only ever adds, which makes it exactly the kind of bias this pipeline
    // already refuses elsewhere.
    //
    // So the path is fitted rather than followed - a running line fit over this
    // much of it, which averages the noise down while leaving curvature at any
    // scale a garment actually has. What the fit removes is reported as
    // roughness_mm rather than discarded.
    double curve_smooth_mm = 4.0;
    // How much of a hand-drawn trace is fitted before it becomes the middle of
    // the corridor.
    //
    // The same argument as the line above, one level up, and it costs more
    // because the noise is larger. A traced path is the operator's hand: a
    // point every few millimetres, each of them a couple of millimetres off the
    // seam in a direction that has nothing to do with the last one. That shake
    // never averages out of an arc length either - it lengthens every segment
    // it touches - and the corridor is built ALONG the traced path, so whatever
    // wiggle is left in it is wiggle in the geometry of the ribbon itself.
    // Nothing downstream can take it out again: the snap is free to move across
    // the corridor and never along it, so a seam found perfectly still comes
    // back measured on a crooked ruler.
    //
    // Measured, on a rendered neckline traced with three millimetres of shake:
    // at 30 mm of window the 254 mm seam read 262, and at 45 it read 254.1.
    // Four traces of one seam agreed to 2.0 mm at 30 and to 0.5 mm at 45. Past
    // about 70 the fit starts cutting the corner instead and the arc turns
    // short, so this sits between.
    //
    // A long window is affordable only because the fit is a QUADRATIC. It
    // follows curvature exactly to second order, so what it leaves behind on a
    // circular arc is the fourth-order term - 0.06 mm across this window on a
    // 91 mm neckline, and under a millimetre on anything a garment curves as
    // tightly as. A running LINE over the same span would cut 4 mm off the
    // corner and flatten the curve it was meant to be following.
    double curve_trace_smooth_mm = 50.0;
    // How much of the corridor's own gradient the winning path has to carry
    // before it counts as a seam at all.
    //
    // A share of what is there rather than an absolute level, because a white
    // rib on a white body has a tenth the contrast of a navy one and no fixed
    // threshold serves both. What this refuses is the case that matters: a
    // corridor with no seam in it still returns a best path, and that path is
    // smooth, plausible, drawn on the operator's screen and completely
    // invented. Better to say the collar cannot be read.
    double curve_min_confidence = 0.60;
    // How much better the line that was measured has to be than the best line
    // somewhere else in the corridor, per sample of its length.
    //
    // The guard against the one failure this design can have that looks exactly
    // like success. Where two real lines lie within the accuracy of a tap - a
    // coverstitch beside the seam it holds down - the winner is decided by
    // which one the arc through the taps happened to land nearer, which is to
    // say by nothing. It comes back smooth, confident, repeatable within a
    // pointing, and different on the next garment by the distance between the
    // two lines.
    //
    // Zero by default, which means the number is reported and never refuses.
    //
    // That is not because the failure is not real - it is, and nothing else in
    // this file can see it. It is because the only numbers this threshold has
    // ever been set against are rendered ones, and on the first real garment it
    // met it refused a lay whose seam had been found at full confidence. A
    // limit calibrated on synthetic cloth and never on a bench is not a limit,
    // it is a guess with an error message, and this one guessed wrong in the
    // direction that stops work.
    //
    // So: run a shift, watch what `margin` actually reads on lays that were
    // good and on lays that were ambiguous, and set this between them. Until
    // then every reading is answered and every margin is written down, which is
    // how the number to set it to gets collected in the first place.
    double curve_min_margin = 0.0;
    // How far the snapped seam may sit from the path the operator traced.
    //
    // Recorded on every reading as drift_mm and refused past this. A snap that
    // has moved two centimetres has not corrected the pointing, it has found
    // something else - the shoulder seam, the edge of a print - and the
    // operator pointing at one thing while the machine measures another is the
    // failure this whole layer exists to avoid.
    double curve_max_drift_mm = 12.0;
    // Shorter than this is not a curve, it is two taps in the same place.
    double curve_min_length_mm = 40.0;
    // How many points the station waits for before it reads the seam.
    //
    // Three, because three is the fewest that describe a curve rather than its
    // two ends, and a neckline is a curve. Two would name the ends of a chord
    // that dips seventy millimetres away from the seam along its whole middle,
    // and the corridor would be searching bare cloth. Set to two for a seam
    // that really is straight; more than three are taken as a trace and
    // followed as they lie.
    //
    // The count also settles when the reading happens without asking, which is
    // what keeps a curve step as quick as a folded one: the third tap is the
    // measurement.
    int curve_seed_count = 3;

    // Whether a curve step reports the arc or the chord.
    //
    // Unlike a fold, where the chord is almost always what the tech pack means,
    // a curve genuinely goes both ways: "neck width" is a chord between two
    // shoulder points, "armhole curved" is the arc along the seam, and the two
    // differ by ten percent on a neckline rather than the millimetre in five
    // hundred a hem manages. So there is no safe default and the style file
    // says which, per step, with this as the fallback. Both are in the log.
    bool curve_measure_along_arc = true;

    // --- The pointer ---
    //
    // How the operator says where the curve is, when they are not tapping the
    // held frame on a tablet.
    //
    // The arm is the only thing in the picture that moves once the garment has
    // settled, so it is found by differencing against the settled frame rather
    // than against the empty mat. That is also what separates it from the
    // shirt: a background difference against the mat returns the shirt and the
    // hand as one blob, and there is no arm in it to find.
    //
    // The difference itself is segmentObject's, so the thresholds that decide
    // what counts as a hand are the chroma and intensity ones already tuned
    // above, and there is no separate one here. That is not tidiness, it is the
    // fix for the thing that broke this on real garments: a hand over pale
    // cloth under a bench light throws a shadow that reaches FURTHER than the
    // hand does and lies in the same direction, so a gray-level threshold takes
    // the far corner of the shadow for a fingertip and puts the point a couple
    // of centimetres past where anybody pointed. The angle between the colour
    // vectors is what tells them apart, and it was already written.
    // A hand is attached to an arm and an arm comes in from outside the
    // picture. A moving blob that touches no border is a shadow, a draught in a
    // hanging sleeve, or the garment settling further - never a hand. This is
    // how far into the frame the border test reaches, in decimated pixels.
    int finger_border_margin_px = 3;
    // The smallest moving thing that could be a hand, as a share of the working
    // area. Below this it is noise; the test is on the decimated frame where a
    // hand is still thousands of pixels.
    double finger_min_area_fraction = 0.0015;

    // -- how the pointer sees a hand --
    //
    // The pointer differences the frame against the settled garment with the
    // same segmentation the garment itself is found with, and the garment's
    // thresholds are wrong for a hand, in a way that is worth being exact about
    // because it is what made the pointer useless on the first coloured shirt.
    //
    // The garment thresholds are tuned to see cloth against a dark mat and to
    // ignore the cloth's own shadow. Brown skin over salmon jersey is neither:
    // it differs from the cloth by five degrees of colour angle against the
    // twelve the garment test wants, and it is seventy percent as bright
    // against a test that only believes "darker" below thirty-five. So the
    // hand is invisible wherever it is over the shirt, visible only where it
    // crosses the mat, and the tip lands where the forearm meets the hem.
    //
    // What actually separates skin from a shadow is not how dark it is but
    // HOW it is dark. A shadow is the same light, less of it: every channel
    // scales by the same factor. Skin over cloth is a different reflectance,
    // and it darkens the blue channel far more than the red - a channel-ratio
    // spread near 0.17 against under 0.05 for a real shadow, on this shirt.
    // That test already exists as shadow_uniformity and it holds at 0.10; the
    // two thresholds in front of it are what have to give. A hand is anything
    // that changed by a few degrees of colour or by fifteen percent of
    // brightness, and is not a shadow.
    double finger_chroma_angle_threshold = 0.008;  // 1 - cos(angle), ~7 degrees
    double finger_intensity_ratio_low = 0.85;
    double finger_intensity_ratio_high = 1.15;
    // How wide a gap in the hand is closed over, so the hand stays attached to
    // the arm that reaches it.
    //
    // There are places where no colour test can see skin: dark skin lying over
    // the dark, warm shadow a hem throws on the mat, or over a fold in salmon
    // cloth, is the same brightness and the same hue as what was there before.
    // Those places are narrow - a fold is five to fifteen millimetres - but a
    // hand is attached to its arm through exactly one of them, at the wrist
    // where the arm crosses the hem, and a hand that has come loose from its
    // arm touches no border and is not a hand as far as the tracker is
    // concerned. Measured on a real held frame with a hand composited on it:
    // the tip then landed at the hem, 150 to 250 mm from the finger. Closing
    // the mask over twice a fold's width reattaches it. It cannot invent a
    // fingertip - closing fills concavities and never extends a convex end.
    double finger_bridge_mm = 20.0;
    // How long the tip has to hold still, and how still, before it counts as a
    // deliberate point rather than the hand passing through.
    //
    // A dwell rather than a button because the other hand is holding fabric.
    // Long enough that reaching across the mat does not scatter seeds behind
    // it; short enough that the operator is not waiting on the machine.
    //
    // Counted in FRAMES as well as seconds, and whichever is longer wins. This
    // bench delivers 1.9 frames a second - a wire limit, sixty megabytes a
    // frame down one gigabit - and at that rate four tenths of a second is
    // barely one frame, which is one sighting, not a hold. Two frames is the
    // fewest that can tell a hand that stopped from a hand that was passing
    // through, and at 1.9 fps that is a second. A camera keeping up gets the
    // four tenths instead. Every wait in the gesture is set this way, because
    // at two frames a second every extra frame is half a second the operator
    // spends looking at a ring.
    double finger_dwell_seconds = 0.4;
    int finger_dwell_frames = 2;
    double finger_dwell_radius_mm = 6.0;
    // How long after a committed point before another one can be taken, so one
    // steady finger does not lay down twenty seeds in the same place.
    double finger_rearm_seconds = 0.8;

    // -- the trace --
    //
    // How far the tip has to move before another point of the trace is kept.
    // Finer than any wobble the corridor cares about, coarse enough that a
    // slow careful trace is not mostly jitter.
    double finger_trace_step_mm = 2.0;
    // How many frames with no hand in them end a trace.
    //
    // A camera looking straight down cannot see a finger lift: the finger is
    // still there, a few millimetres higher and slightly displaced. So what
    // ends a trace is the hand LEAVING, and it has to be gone for two frames
    // rather than one, because a hand crossing its own shadow drops out for a
    // frame and comes back. Two rather than three for the reason above: at
    // this frame rate each one is half a second of the operator waiting.
    int finger_trace_lift_frames = 2;
    // How long the tip may stand still before that ENDS a trace, or zero for
    // never.
    //
    // Off by default, and the reason is the gesture as a whole. Hold, draw,
    // lift is one motion with one wait in it; hold, draw, hold-for-three-
    // seconds is two, and the second one is the longer. And a hand drawing a
    // seam slows almost to a stop all the time - at a corner, at the deepest
    // part of an arc, whenever the operator looks up - so any still-hold short
    // enough to be worth having ends the trace halfway along the collar for no
    // reason the operator can see. Taking the hand away is unambiguous and it
    // is what an operator does anyway. Set this for a bench where the operator
    // cannot lift clear of the picture.
    double finger_trace_end_seconds = 0.0;
    // The fastest a hand draws, and so the furthest the tip may move between
    // one frame and the next before the move is a lost finger rather than a
    // stroke.
    //
    // Tracing is done at thirty to a hundred millimetres a second; three
    // hundred is a hand moving too fast to be drawing anything. A tip that has
    // moved further than that since the last frame - to the elbow, to the far
    // corner of a shadow, to a fold in the shirt that lifted when the hand
    // brushed it - is not recorded. It is not a point on the seam, and one
    // such point is a spike the corridor cannot recover from. Scaled by the
    // actual interval between the two frames, so it means the same thing at
    // two frames a second as at ten.
    //
    // And forgiven after finger_lost_frames of it in a row, because a tracker
    // that will never accept a big move is a tracker that can be wedged on a
    // wrong place forever. Two frames of the tip somewhere else, agreeing with
    // each other, is the hand really being there.
    double finger_max_speed_mm_s = 300.0;
    int finger_lost_frames = 2;

    // How far above the mat a pointing fingertip sits, and so how much parallax
    // to take out of it.
    //
    // The plane model is a homography onto the mat; a fingertip is not on the
    // mat. A point at this height, this far out from the middle of the picture,
    // images displaced outward by (radius x height / working distance) - four
    // millimetres at the edge of this frame - and the displacement is smooth,
    // growing from nothing at the centre. That makes it a BIAS rather than
    // noise, which matters: smoothing a trace averages away shake and does
    // nothing whatever to this. Left in, it moves the whole corridor toward
    // whichever side of the picture the garment sits on, and where a second
    // line lies a few millimetres from the seam it decides which one gets
    // measured.
    //
    // Correcting it costs one multiplication and needs the working distance,
    // which comes from the lens focal length: pass --focal-length-mm and it is
    // derived, and without it the correction is skipped and said so out loud.
    double finger_height_mm = 10.0;

    // How much of the hand, around its furthest point, counts as the fingertip
    // when working out how wide that fingertip is - and the most the reported
    // tip may be moved back along the arm as a result.
    //
    // The tip is reported half a fingertip behind the leading edge of the
    // silhouette, because that is where the pad is and the pad is what touches.
    // How far back is measured from the picture rather than assumed, and
    // measuring it wrong is not a small error: the window used to be a fraction
    // of how far the arm reached into the frame, which on a real arm is seventy
    // millimetres, so what got measured was the width of the HAND rather than
    // of the fingertip - thirty-five millimetres of fist - and the tip was set
    // back into the middle of the hand. It tracked the hand and the trace
    // wandered around the knuckles.
    //
    // Fifteen millimetres is about a fingertip. Ten is about half of one, and
    // capping the setback there is what bounds the damage when somebody points
    // with a closed fist and there is no fingertip in the picture to find.
    double finger_tip_look_mm = 15.0;
    double finger_tip_setback_max_mm = 10.0;

    // --- Collar detection ---
    //
    // A "collar" step is read from a box the operator draws round the collar.
    // The box says where the collar is and nothing else: the lines, the edge
    // they are measured on and the two points they are measured between all
    // come from the picture. See collar.hpp and docs/collar_detection.md.
    //
    // Nothing here is read by any other kind of step, and the curve settings
    // above are used by it only through snapCurve, unchanged.
    //
    // How far either side of each line of the collar the final read searches.
    //
    // Narrow on purpose. By the time a line is read to a fraction of a
    // millimetre its place is already known to about one, from the profile
    // across the whole band, and the corridor's job is then to hold THAT line
    // rather than to look for one. A topstitch three millimetres beside the
    // attachment seam is inside this and loses to the seam on the prior, and
    // the band's other edge fifteen millimetres away is not in it at all.
    double collar_corridor_mm = 6.0;
    // How far a line may be carried on past where the picture last showed it,
    // to meet the edge of the cloth.
    //
    // A collar's seam runs to the edge of a shirt laid flat, and where it gets
    // there is the shoulder point. A line that has to be carried further than
    // this to reach the edge does not get there at all: it is a fold or a
    // shadow that stopped, and it is not offered as an edge of the collar.
    double collar_edge_reach_mm = 12.0;
    // How far either side of the outline's edge the picture is searched for
    // where the cloth really ends, along the line.
    //
    // The outline is a guide and not the answer. Glow off pale cloth pushes it
    // several millimetres out onto the mat, and a collar measured to that
    // outline would read long by the same amount at both ends. The fall from
    // cloth to mat is the strongest change of shade anywhere near the end of a
    // collar line, and this is how far from the outline it is looked for.
    double collar_edge_search_mm = 8.0;
    // How far apart the band's two edges may be for a pair of lines to be read
    // as the collar band at all. A rib collar is twelve to twenty-five
    // millimetres. Two crisp lines further apart or closer together than this
    // are two things that happen to lie side by side - a seam and the stitch
    // beside it are closer, a seam and a fold in the body usually further -
    // and are not a band.
    double collar_band_min_mm = 8.0;
    double collar_band_max_mm = 35.0;
    // A collar on a dark garment is read from a brightened picture.
    //
    // Every line of a collar is held to floors in gray levels as well as to
    // the picture's own noise, and they were set on pale shirts, where a
    // collar's edges are tens of levels deep. On a black shirt the whole
    // collar sits between one and nine levels and its edges one or two apart,
    // under every floor, and nothing of it was found. So when the middle gray
    // level of the cloth in the box is below collar_dark_below, the collar is
    // read from the picture scaled until the cloth sits at collar_dark_level.
    // Rendered at a thirtieth of a pale shirt's brightness, the collar this is
    // tested on went from 25 checks failed of 40 to 5 of 59. A cloth brighter
    // than collar_dark_below is read from the same picture as before, to the
    // bit. 0 turns this off.
    double collar_dark_below = 24.0;
    double collar_dark_level = 80.0;

    // --- The full perimeter ---
    //
    // A "perimeter" step measures the whole outer edge of a garment laid flat.
    // A shirt is bigger than the view, so it is taken in two halves: two clips
    // mark where the halves meet, one on the middle of the back neck and one on
    // the middle of the hem, and the station cuts each half's outline at the
    // clips and adds the two. See perimeter.hpp and docs/garment_perimeter.md.
    //
    // Nothing here is read by any other kind of step.
    //
    // The side of the working area the join band runs along: the other half of
    // the shirt lies past it. The bottom, because on this bench that is the
    // longest edge of the view, and the length of a half has to fit along it.
    std::string perimeter_join_side = "bottom";
    // How far in from that edge the clips have to sit, in millimetres. Far
    // enough in that the cloth is still in view either side of each clip, near
    // enough that half a shirt still fits across the rest of the view.
    double perimeter_band_near_mm = 15.0;
    double perimeter_band_far_mm = 60.0;
    // The margin the guide asks to be kept clear on the other three sides. The
    // guide only: a half is refused when its outline actually touches one of
    // those edges, not when it comes inside this.
    double perimeter_margin_mm = 10.0;
    // How close to the border of the working area an outline point has to be
    // to count as touching it, in full-resolution pixels. A garment that runs
    // off the picture is cut by the border, and the refinement can move those
    // points up to subpixel_search_px either way, on a boundary resampled every
    // contour_resample_px, so this has to cover both.
    double perimeter_border_touch_px = 10.0;

    // The clips, by colour. Hue on the usual 0 to 360 degree wheel, and a low
    // bound above a high one wraps through red. Yellow by default because
    // nothing else on this bench is: the mat is dark green, the shirts are pale
    // or printed in red and black. A step can name its own with
    // "clip_hue_deg": [lo, hi] for a garment the clips would not stand out on.
    double perimeter_clip_hue_lo_deg = 40.0;
    double perimeter_clip_hue_hi_deg = 70.0;
    int perimeter_clip_sat_min = 110;  // 0-255: a clip is a strong colour
    int perimeter_clip_val_min = 90;   // 0-255: and not in shadow
    // How big a clip looks from above, in square millimetres. A sewing clip is
    // about 300; this admits a small peg and a large bulldog clip and nothing
    // the size of a print or a speck.
    double perimeter_clip_area_min_mm2 = 30.0;
    double perimeter_clip_area_max_mm2 = 3000.0;
    // How far round each clip the outline is taken out and bridged straight
    // across. The clip is foreground to the segmentation, so the outline bulges
    // round it; its shadow can too. A straight bridge across 20 mm of hem or
    // back neck is within a hundredth of a millimetre of the cloth under it.
    double perimeter_clip_clear_mm = 6.0;
    // How close to the outline a clip has to be to count as on the edge of the
    // shirt, rather than lying on the cloth or the mat.
    double perimeter_clip_on_edge_mm = 4.0;

    // The traced length: the cloth followed, measured every this many
    // millimetres. Logged beside the fitted length; see garment_perimeter.md.
    double perimeter_trace_step_mm = 5.0;
    // The fitted length, which is what a pattern draws: smooth seam lines that
    // meet at sharp corners. A corner is where the outline turns by at least
    // perimeter_corner_min_turn_deg within perimeter_corner_window_mm. The two
    // edges either side of it are fitted over perimeter_corner_fit_mm and
    // carried on until they cross, no further than perimeter_corner_reach_mm.
    double perimeter_corner_window_mm = 30.0;
    double perimeter_corner_min_turn_deg = 30.0;
    // How sharply the outline has to be curving to count as part of a corner
    // rather than part of an edge, in degrees per millimetre. A back neck
    // curves at about 0.3, a rounded underarm at about 3.
    double perimeter_corner_curvature_deg_per_mm = 1.0;
    double perimeter_corner_fit_mm = 40.0;
    double perimeter_corner_fit_min_mm = 15.0;
    // Bounded, because a crossing is a computed answer and not a measurement.
    // An underarm rounded to 20 mm meets its crossing about 50 mm out.
    double perimeter_corner_reach_mm = 80.0;
    // How far an edge may wander from one smooth curve before it counts as
    // shape and is fitted in pieces, RMS. Below this the wander is the cloth,
    // and a pattern has none of it.
    double perimeter_edge_tolerance_mm = 1.0;
    double perimeter_edge_piece_min_mm = 40.0;
    // How long a notch or a spike may be, along the outline, and still be
    // bridged straight across as a defect rather than followed: where the
    // boundary turns hard and turns back again within this, it went somewhere
    // and came back - a thread, a label, a printed line of the mat the
    // segmentation followed into the cloth - and the garment's edge ran
    // straight on underneath.
    double perimeter_defect_span_mm = 40.0;

    // --- Working through the step list ---
    // A step that has passed is finished. Waiting for a hand to reach the Next
    // button costs a second of every step, and that hand is usually holding
    // fabric, so the station moves on by itself once a step passes.
    //
    // Only on a pass. A fail stays where it is: the operator has to see it,
    // decide, and either refold and retake or accept it and move on. Advancing
    // off a failure would turn a rejected garment into a shrug.
    bool auto_advance_on_pass = true;
    // How long the PASS stays on screen before the next step replaces it. Long
    // enough to read the verdict that justified the move, short enough not to
    // be the thing the operator is waiting for. Next still works during it, and
    // Back still works after it.
    //
    // The page spends it in two parts: the step shown as complete, then the
    // move to the next one counted down. At two seconds each part was under a
    // second, which reads as the panel jumping rather than moving on. Three
    // costs nothing at the bench - the garment has to be picked up and
    // refolded anyway, and no new reading can arrive in under about six.
    double auto_advance_delay_s = 3.0;

    // --- The working area ---
    // How far past the last detected grid line the station may work, in grid
    // pitches.
    //
    // The printed grid is what the plane model was fitted to, but it is not the
    // limit of what the camera can see, and the reach along the rail is the
    // hard limit on what this bench can measure at all: a 710 mm body length
    // does not fit in a 692 mm working area at any tolerance, and the reading
    // that comes back is the width of the view wearing a measurement's clothes.
    // So the working area is allowed to run out past the grid, as far as the
    // picture goes. The model extrapolates a few pitches perfectly well; it is
    // a global fit to hundreds of points, not an interpolation between them.
    double roi_extrapolation_pitches = 4.0;
    // ...but never quite to the border. The last pixels of a frame carry the
    // vignetting and the worst of the lens, and a rail drawn on the very edge
    // of the picture cannot be told from the edge of the picture.
    int roi_frame_margin_px = 8;

    // Reach to the edge of the mat, rather than to a fixed count of pitches
    // past the grid.
    //
    // A few pitches past the lattice is a guess at where the mat ends, and it
    // is only ever right by accident: the printed grid stops well short of the
    // mat's border on some mats and runs almost to it on others, so the same
    // constant throws away 150 mm of reach on one bench and overshoots onto the
    // bench top on the next. The mat itself is the thing with the real answer,
    // and it is visible in the very frame the calibration was taken from, so it
    // is measured rather than assumed.
    //
    // The grid still has the last word. What the mat is allowed to do is push
    // the working area outward, never pull it in: the box below is the union of
    // the extrapolated lattice and the detected mat, so turning this on can
    // only ever gain reach, and a segmentation that finds nothing leaves the
    // previous behaviour exactly as it was.
    bool roi_follow_mat = true;
    // How far past the lattice the mat is allowed to push, in grid pitches.
    //
    // This is the guard against a segmentation that leaks off the mat and onto
    // the bench, and it is a guard worth having for a second reason: the plane
    // model was fitted over the lattice and nowhere else, so every pixel beyond
    // it is extrapolation. A few dozen pitches out from a fit over hundreds of
    // points is still sound. Several hundred is not, and a rail drawn there
    // would report millimetres it cannot stand behind.
    double roi_mat_max_pitches = 60.0;
    // How far a pixel's colour may sit from the mat's own before it stops
    // counting as mat, in robust standard deviations of the colour found
    // inside the grid. Raise it if the mat's printed markings are being cut
    // off; lower it if the working area is running onto the bench.
    double roi_mat_colour_sigmas = 4.5;

    // --- Object geometry ---
    double object_height_mm = 0.0;               // top-surface height above the mat
    std::optional<double> camera_distance_mm;    // auto-derived if focal length is given

    // --- Display ---
    double display_scale = 0.25;

    // Tidying the drawn outline. Display only.
    //
    // Nothing in this section reaches a measurement. The boundary a measurement
    // is taken from goes to measureEdge exactly as the refinement produced it,
    // spikes and all; what is drawn is a copy, tidied, because a jagged line
    // around a straight edge tells the operator nothing except that the
    // rendering is noisy. If these two disagreed with the measurement the
    // overlay would be a lie, so they are applied to the copy and nowhere else.
    //
    // How much of the boundary the tidying looks at, as a length along the
    // outline in display pixels.
    //
    // A length rather than a count of samples, because the two outlines that
    // get drawn are sampled differently: the held one every contour_resample_px
    // of the full-resolution frame, the live one every decimated cell. A count
    // would smooth one of them twice as hard as the other, and the live outline
    // is the one that needs it most. A length treats them the same.
    //
    // Wider is smoother and rounds a corner more. This is about the width of
    // the artefacts the mask leaves, which is what it has to be to remove them.
    double display_outline_smooth_px = 20.0;

    // Straightening what is left into line segments. Off, and worth saying why.
    //
    // Douglas-Peucker is a simplifier, not a smoother. Handed a boundary that
    // still carries artefacts it does not remove them, it renders each one as a
    // crisp triangle with a vertex at its tip: the outline stops looking noisy
    // and starts looking torn, which is worse, because noise reads as noise and
    // a torn line reads as a machine that has found the wrong edge.
    //
    // The smoothing above is what makes a straight edge straight, and it does
    // it by re-estimating where the boundary is rather than by cutting corners
    // off it. Raise this only to trade fidelity for fewer segments.
    double display_outline_simplify_px = 0.0;

    // --- Step pictures ---
    //
    // A picture of every verdict, pass and fail, as the feed showed it with a
    // band of what was judged, filed under
    //   <capture_dir>/PASS|FAIL|MANUAL/YYYY-MM-DD/HHMMSS_g<shirt>_s<step>_<style>.jpg
    // and shown again beside the feed when the operator goes back to the step.
    // Written by a thread of its own: nothing here reaches a measurement, and
    // the measuring thread only hands over the picture it already holds.
    bool capture_enabled = true;
    std::string capture_dir = "captures";
    int capture_jpeg_quality = 85;
    // The size the picture is written at, as a fraction of the camera's. Zero
    // is the feed's own size (display_scale), strokes and all.
    double capture_scale = 0.0;
    // Pictures waiting to be written. When full the oldest waiting one is
    // dropped, and said, rather than hold up the measuring.
    int capture_queue = 8;
    // How much is kept: whole days, oldest first, until both hold. Zero is
    // no limit. Only ever *.jpg inside the date folders under capture_dir.
    int capture_keep_days = 0;
    double capture_max_gb = 0.0;
    // Checked when the station starts, then after this many pictures or an
    // hour, whichever comes first.
    int capture_prune_every = 50;
};

}  // namespace verimeasure
