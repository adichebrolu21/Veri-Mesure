// A measurement template: one garment style, as an ordered list of steps.
//
// One step is one thing the operator does. Fold the shirt the way the step's
// picture shows, lay the fold against the rail, let the station read it, take
// the pass or the fail, move on. Four to six of those and the shirt is done.
//
// So the file is a list, in the order the operator works through it, and each
// entry carries only what that one step needs: what to call it, what it should
// come to, how far out it may be, a picture of the fold and a line of words.
//
//   {
//     "style": "style-42-polo-M",
//     "steps": [
//       {
//         "name": "half_chest",
//         "target_mm": 492,
//         "tol_minus_mm": -12.7,
//         "tol_plus_mm": 12.7,
//         "reference_image": "refs/half_chest.svg",
//         "instruction": "Fold along the side seam, align the fold to the left edge"
//       }
//     ]
//   }
//
// Two things are deliberately absent.
//
// There is no datum position. Nothing in this file records where anything sits
// on the table, because a stored position is the one thing a mat sliding under
// the camera would invalidate. The span is taken between two edges of the
// fabric itself, so the file holds no coordinate to go stale.
//
// And tolerances are absolute and two-sided, not a percentage. A tech pack that
// allows 1/2" on a body length and 1/8" on a shoulder is not describing one
// proportional rule: as percentages those are 1.8% and 25% of their own
// targets. They are also often lopsided, so the minus and plus limits are
// stored separately rather than as one half-width.
//
// Lengths may be written as numbers or as the fractional inches a tech pack
// uses: "27 1/2", "-1/8", "3/8". The file states its units once, with "units",
// and every length in it is read that way. Everything downstream is millimetres.

#pragma once

#include <string>
#include <vector>

#include "verimeasure/json.hpp"

namespace verimeasure {

struct Step {
    std::string name;         // shown on the operator display and in the log
    double target_mm = 0.0;
    double tol_minus_mm = 0.0;  // zero or negative
    double tol_plus_mm = 0.0;   // zero or positive

    // How this step is taken. "edge" is the default and everything above
    // describes it: fold the dimension into one straight edge, lay it against
    // the rail, the machine reads it.
    //
    // "curve" is for the dimensions that cannot be folded into an edge because
    // they are not on the outline of the cloth at all. A neckline is the case
    // this was written for: the collar meets the body somewhere inside the
    // shirt, the neck opening shows the back panel rather than the mat, and no
    // amount of folding turns that seam into a silhouette. On a curve step the
    // operator points at the seam - three taps, on the tablet or in the air -
    // and the machine finds it in the picture between them.
    //
    // Kept in the style file rather than set for the whole bench because one
    // shirt has both kinds of step in it, and the operator has to be told which
    // one they are on.
    // "distance" is the plainest thing this bench can do: the operator taps
    // two points on the held picture and the machine reports the straight
    // distance between them, exactly where they were put, through the
    // calibration. No seam is looked for and nothing is corrected. It is a
    // ruler laid on the picture, and it exists because there are dimensions
    // that are simply two landmarks apart - a neck width from shoulder point to
    // shoulder point across the opening, a placket length - where there is no
    // seam between the ends for a curve step to snap to, and no edge to fold.
    // What it measures is where the operator pointed, so its accuracy is the
    // operator's aim on the picture: about half a millimetre on this feed.
    //
    // "collar" is a neckline read with the operator taken out of it: they
    // draw a box round the collar on the held picture and the machine finds
    // the collar band's two edges inside it, runs each to the edge of the
    // cloth - the shoulder points - and measures the one "seam" names. The
    // box says where the collar is and nothing more; drawn differently it
    // gives the same number. See collar.hpp.
    //
    // "perimeter" is the whole outer edge of the garment laid flat, taken in
    // two halves because a shirt is bigger than the view: two clips mark where
    // the halves meet, one half is laid in view with the clips in a band along
    // the bottom edge, then the shirt is turned end for end for the other. It
    // is read from the engine's readings the way an edge step is - nothing is
    // pointed at - so it is not a pointed step. See perimeter.hpp.
    std::string kind = "edge";
    bool isCurve() const { return kind == "curve"; }
    bool isDistance() const { return kind == "distance"; }
    bool isCollar() const { return kind == "collar"; }
    bool isPerimeter() const { return kind == "perimeter"; }
    // Any of the three taken from the held picture rather than by laying an
    // edge against the rail.
    bool isPointed() const { return isCurve() || isDistance() || isCollar(); }

    // Which length a curve step reports: "arc" along the seam, or "chord"
    // across it. Empty follows cfg.curve_measure_along_arc.
    //
    // A fold hardly cares - the chord and the trace differ by a millimetre in
    // five hundred - so the folded edge has one global setting and that is
    // enough. A neckline is not like that. Its arc runs forty percent longer
    // than its chord, so "neck width" and "neckline seam length" are different
    // measurements of the same seam and a file that does not say which has not
    // specified the step.
    //
    // On a perimeter step it is "fitted" (the default: edges fitted, corners
    // crossed, the way a pattern draws the outline) or "traced" (the cloth
    // followed, measured every perimeter_trace_step_mm).
    //
    // A collar step also takes "around": the whole collar, the band's outer
    // edge over the front from one shoulder point to the other and the edge of
    // the shirt across the back of the neck between the same two points.
    std::string measure;

    // The colour of the clips a perimeter step joins its halves at, as a hue
    // range in degrees, for a garment the bench's own clips would not stand
    // out on. Negative means the bench's setting (perimeter_clip_hue_*_deg).
    double clip_hue_lo_deg = -1.0;
    double clip_hue_hi_deg = -1.0;
    bool hasClipHue() const { return clip_hue_lo_deg >= 0.0 && clip_hue_hi_deg >= 0.0; }

    // How a curve step is found. "auto", the default, lets the machine find
    // the neckline from the garment's own outline the moment the reading
    // holds and read it without being pointed at, falling back to pointing
    // whenever it is not sure. "point" waits for the operator every time - for
    // a garment whose neck is not a notch in its outline, or a bench that
    // would rather see the pointing.
    std::string find;
    bool autoFind() const { return isCurve() && find != "point"; }

    // Which edge of a collar band the step means, when the machine finds the
    // band by itself: "outer" (the default) is the attachment seam, where the
    // band meets the body - the larger horseshoe; "inner" is the neck
    // opening's own edge, a band's width inside it. Both are real, strong,
    // equally readable lines, and nothing in the picture says which the tech
    // pack wants - only the step can. A drawn or tapped pointing makes the
    // choice itself and ignores this.
    std::string seam;

    // How thick the cloth stands off the mat where this step is read, in
    // millimetres. Zero means the bench's own setting is used.
    //
    // This is per step because it is not a property of the garment, it is a
    // property of the fold. A bottom hem laid as one clean edge is two layers
    // of jersey and a hem allowance - three or four millimetres. The same shirt
    // folded down its centre for a half-chest is four layers over a sleeve
    // that is itself two, and stands three times that.
    //
    // It matters more than its size suggests, because the camera is looking
    // down at the top face and the top face is closer to the lens than the mat
    // the calibration describes. It therefore images larger, by d / (d - h),
    // and that is a scale error on the whole measurement rather than an offset:
    // at this bench's three-quarters of a metre an 11 mm fold reads 1.5% long,
    // which is 3.2 mm on a sleeve and 8 mm on a body length. Rendered and
    // measured through this exact pipeline, a 225 mm rectangle at 11 mm height
    // came back 228.21 mm uncorrected and 224.84 mm corrected.
    //
    // Measure it once per step with a caliper on a garment folded the way the
    // step asks for, and it is right for every garment of that cloth.
    double height_mm = 0.0;

    std::string reference_image;  // path, relative to the templates directory
    std::string instruction;      // how to fold and lay this one

    double low_mm() const { return target_mm + tol_minus_mm; }
    double high_mm() const { return target_mm + tol_plus_mm; }
};

struct Template {
    std::string style;
    std::string source_units = "mm";  // as written in the file
    std::string path;                 // where it was read from, so edits go back
    std::vector<Step> steps;
};

// One template on disk, as the operator page offers it for selection.
struct TemplateEntry {
    std::string style;
    std::string path;
    int steps = 0;
};

// Every readable template in a directory, sorted by style. Files that are not
// valid templates are skipped rather than reported: the directory is a working
// folder on a factory machine and will collect stray JSON.
std::vector<TemplateEntry> listTemplates(const std::string& dir);

bool loadTemplate(const std::string& path, Template& out, std::string& error);
bool parseTemplate(const vjson::Value& v, Template& out, std::string& error);
vjson::Value templateToJson(const Template& t);

// Write the template back where it came from.
//
// Targets and tolerances are edited from the operator page, so this is how a
// correction made at the bench survives a restart. Always written in
// millimetres and marked as such, whatever units the file was first authored
// in: silently re-deriving fractional inches from a millimetre value the
// operator typed would be a quiet invention, and a file that says "mm" and
// holds millimetres cannot be misread later.
bool saveTemplate(const Template& t, const std::string& path, std::string& error);

// "27 1/2" -> 698.5 with units "in". Accepts a JSON number or a string, a bare
// decimal, a bare fraction, or a whole part and a fraction. Returns false with
// a reason on anything else.
bool parseLength(const vjson::Value& v, const std::string& units, double& mm,
                 std::string& error);

}  // namespace verimeasure
