#include "verimeasure/template.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <system_error>

namespace verimeasure {
namespace {

std::string trim(const std::string& s) {
    std::size_t a = 0;
    std::size_t b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) {
        --b;
    }
    return s.substr(a, b - a);
}

std::vector<std::string> splitWhitespace(const std::string& s) {
    std::vector<std::string> out;
    std::size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
        const std::size_t start = i;
        while (i < s.size() && s[i] != ' ' && s[i] != '\t') ++i;
        if (i > start) out.push_back(s.substr(start, i - start));
    }
    return out;
}

bool parseDecimal(const std::string& s, double& out) {
    if (s.empty()) return false;
    const char* begin = s.c_str();
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin || *end != '\0') return false;
    out = v;
    return true;
}

// "1/2" -> 0.5. Only a plain positive ratio; the sign is handled by the caller.
bool parseRatio(const std::string& s, double& out) {
    const std::size_t slash = s.find('/');
    if (slash == std::string::npos) return false;
    double num = 0.0;
    double den = 0.0;
    if (!parseDecimal(s.substr(0, slash), num)) return false;
    if (!parseDecimal(s.substr(slash + 1), den)) return false;
    if (den == 0.0) return false;
    out = num / den;
    return true;
}

// The forms a tech pack actually writes: "27 1/2", "1/2", "-1/8", "19.375", "8".
bool parseMixedNumber(const std::string& raw, double& out, std::string& error) {
    std::string s = trim(raw);
    if (s.empty()) {
        error = "empty length";
        return false;
    }
    double sign = 1.0;
    if (s[0] == '+' || s[0] == '-') {
        if (s[0] == '-') sign = -1.0;
        s = trim(s.substr(1));
    }
    const std::vector<std::string> tokens = splitWhitespace(s);
    if (tokens.empty()) {
        error = "empty length";
        return false;
    }
    if (tokens.size() == 1) {
        double v = 0.0;
        if (parseRatio(tokens[0], v) || parseDecimal(tokens[0], v)) {
            out = sign * v;
            return true;
        }
        error = "\"" + raw + "\" is not a number or a fraction";
        return false;
    }
    if (tokens.size() == 2) {
        double whole = 0.0;
        double frac = 0.0;
        if (parseDecimal(tokens[0], whole) && parseRatio(tokens[1], frac)) {
            out = sign * (whole + frac);
            return true;
        }
        error = "\"" + raw + "\" is not a whole number followed by a fraction";
        return false;
    }
    error = "\"" + raw + "\" has too many parts to be a length";
    return false;
}

bool unitScale(const std::string& units, double& mmPerUnit, std::string& error) {
    if (units == "mm") {
        mmPerUnit = 1.0;
        return true;
    }
    if (units == "cm") {
        mmPerUnit = 10.0;
        return true;
    }
    if (units == "in" || units == "inch" || units == "inches") {
        mmPerUnit = 25.4;
        return true;
    }
    error = "unknown units \"" + units + "\"; expected in, mm or cm";
    return false;
}

// A field whose name ends in _mm is already millimetres whatever the file says
// its units are, because the name says so. Anything else is read in the file
// units. That is what lets the plain millimetre form and a tech pack written in
// fractional inches both be valid files.
bool readLength(const vjson::Value& parent, const char* mmKey, const char* plainKey,
                const std::string& units, bool required, double& out, std::string& error) {
    if (const vjson::Value* v = parent.find(mmKey)) {
        return parseLength(*v, "mm", out, error);
    }
    if (const vjson::Value* v = parent.find(plainKey)) {
        return parseLength(*v, units, out, error);
    }
    if (required) {
        error = std::string("missing \"") + mmKey + "\"";
        return false;
    }
    out = 0.0;
    return true;
}

}  // namespace

bool parseLength(const vjson::Value& v, const std::string& units, double& mm,
                 std::string& error) {
    double mmPerUnit = 1.0;
    if (!unitScale(units, mmPerUnit, error)) return false;

    if (v.isNumber()) {
        mm = v.asNumber() * mmPerUnit;
        return true;
    }
    if (v.type == vjson::Value::Type::Str) {
        double value = 0.0;
        if (!parseMixedNumber(v.s, value, error)) return false;
        mm = value * mmPerUnit;
        return true;
    }
    error = "a length must be a number or a string";
    return false;
}

bool parseTemplate(const vjson::Value& v, Template& out, std::string& error) {
    out = Template();
    if (!v.isObject()) {
        error = "a template must be a JSON object";
        return false;
    }

    out.style = v.getString("style");
    if (out.style.empty()) {
        error = "missing \"style\"";
        return false;
    }

    out.source_units = v.getString("units", "mm");
    double mmPerUnit = 1.0;
    if (!unitScale(out.source_units, mmPerUnit, error)) return false;

    const vjson::Value* steps = v.find("steps");
    if (!steps || !steps->isArray() || steps->size() == 0) {
        error = "a template needs a non-empty \"steps\" array";
        return false;
    }

    for (std::size_t i = 0; i < steps->size(); ++i) {
        const vjson::Value& s = steps->at(i);
        const std::string where = "step " + std::to_string(i + 1);
        if (!s.isObject()) {
            error = where + " is not an object";
            return false;
        }

        Step step;
        step.name = s.getString("name");
        if (step.name.empty()) {
            error = where + " has no \"name\"";
            return false;
        }
        step.kind = s.getString("kind", "edge");
        if (step.kind != "edge" && step.kind != "curve" && step.kind != "distance" && step.kind != "collar" &&
            step.kind != "perimeter") {
            error = where + " (" + step.name + ") has a \"kind\" this build does not know: " +
                    step.kind;
            return false;
        }
        step.find = s.getString("find");
        if (!step.find.empty() && step.find != "auto" && step.find != "point") {
            error = where + " (" + step.name +
                    ") can be found by the machine (\"auto\") or by pointing (\"point\"), not: " +
                    step.find;
            return false;
        }
        step.seam = s.getString("seam");
        if (!step.seam.empty() && step.seam != "outer" && step.seam != "inner") {
            error = where + " (" + step.name +
                    ") can mean the \"outer\" edge of a band or the \"inner\", not: " + step.seam;
            return false;
        }
        step.measure = s.getString("measure");
        if (step.isPerimeter()) {
            // A perimeter is either the pattern's, with the corners crossed, or
            // the cloth's own, followed.
            if (!step.measure.empty() && step.measure != "fitted" && step.measure != "traced") {
                error = where + " (" + step.name +
                        ") measures the perimeter \"fitted\", as a pattern draws it, or \"traced\" "
                        "along the cloth, not: " + step.measure;
                return false;
            }
        } else if (step.isCollar() && step.measure == "around") {
            // The whole collar: the band's outer edge over the front and the
            // edge of the shirt across the back, which is where the outer edge
            // ends. The opening's edge ends somewhere else.
            if (step.seam == "inner") {
                error = where + " (" + step.name +
                        ") measures the collar \"around\", which is along its outer edge, so its \"seam\" "
                        "cannot be \"inner\"";
                return false;
            }
        } else if (!step.measure.empty() && step.measure != "arc" && step.measure != "chord") {
            error = where + " (" + step.name +
                    ") must measure along the \"arc\" or across the \"chord\"" +
                    (step.isCollar() ? std::string(", or \"around\" the whole collar") : std::string()) +
                    ", not: " + step.measure;
            return false;
        }
        if (const vjson::Value* hue = s.find("clip_hue_deg")) {
            if (!hue->isArray() || hue->size() != 2 || !hue->at(0).isNumber() || !hue->at(1).isNumber() ||
                hue->at(0).asNumber() < 0.0 || hue->at(0).asNumber() > 360.0 ||
                hue->at(1).asNumber() < 0.0 || hue->at(1).asNumber() > 360.0) {
                error = where + " (" + step.name +
                        "): \"clip_hue_deg\" is two hues in degrees, 0 to 360, such as [40, 70]";
                return false;
            }
            step.clip_hue_lo_deg = hue->at(0).asNumber();
            step.clip_hue_hi_deg = hue->at(1).asNumber();
        }

        step.reference_image = s.getString("reference_image");
        step.instruction = s.getString("instruction");

        std::string why;
        // Optional, and written the same way every other length in the file is,
        // so a tech pack in fractional inches can say "3/16" and mean it.
        if (!readLength(s, "height_mm", "height", out.source_units, false, step.height_mm, why)) {
            error = where + " (" + step.name + "): " + why;
            return false;
        }
        if (!readLength(s, "target_mm", "target", out.source_units, true, step.target_mm, why)) {
            error = where + " (" + step.name + "): " + why;
            return false;
        }
        if (!readLength(s, "tol_minus_mm", "tol_minus", out.source_units, true, step.tol_minus_mm,
                        why)) {
            error = where + " (" + step.name + "): " + why;
            return false;
        }
        if (!readLength(s, "tol_plus_mm", "tol_plus", out.source_units, true, step.tol_plus_mm,
                        why)) {
            error = where + " (" + step.name + "): " + why;
            return false;
        }

        if (step.target_mm <= 0.0) {
            error = where + " (" + step.name + "): the target must be a positive length";
            return false;
        }

        // A file that writes both limits positive means a symmetric band and is
        // saying so clumsily, not asking for a band that starts above the
        // target. Taking that at face value would quietly fail every good
        // garment, so the signs are normalised rather than trusted.
        step.tol_minus_mm = -std::fabs(step.tol_minus_mm);
        step.tol_plus_mm = std::fabs(step.tol_plus_mm);

        out.steps.push_back(step);
    }

    return true;
}

bool loadTemplate(const std::string& path, Template& out, std::string& error) {
    vjson::Value doc;
    if (!vjson::parseFile(path, doc, error)) return false;
    if (!parseTemplate(doc, out, error)) return false;
    out.path = path;
    return true;
}

vjson::Value templateToJson(const Template& t) {
    vjson::Value o = vjson::Value::object();
    o.set("style", vjson::Value::str(t.style));
    o.set("units", vjson::Value::str("mm"));

    vjson::Value steps = vjson::Value::array();
    for (const Step& s : t.steps) {
        vjson::Value e = vjson::Value::object();
        e.set("name", vjson::Value::str(s.name));
        e.set("target_mm", vjson::Value::real(s.target_mm));
        e.set("tol_minus_mm", vjson::Value::real(s.tol_minus_mm));
        e.set("tol_plus_mm", vjson::Value::real(s.tol_plus_mm));
        // Written only when it is not the default, so a bench that never uses
        // curve steps keeps the files it already has, byte for byte.
        if (s.kind != "edge") e.set("kind", vjson::Value::str(s.kind));
        if (!s.measure.empty()) e.set("measure", vjson::Value::str(s.measure));
        if (!s.find.empty()) e.set("find", vjson::Value::str(s.find));
        if (!s.seam.empty()) e.set("seam", vjson::Value::str(s.seam));
        if (s.height_mm != 0.0) e.set("height_mm", vjson::Value::real(s.height_mm));
        if (s.hasClipHue()) {
            vjson::Value hue = vjson::Value::array();
            hue.push(vjson::Value::real(s.clip_hue_lo_deg));
            hue.push(vjson::Value::real(s.clip_hue_hi_deg));
            e.set("clip_hue_deg", hue);
        }
        if (!s.reference_image.empty()) {
            e.set("reference_image", vjson::Value::str(s.reference_image));
        }
        if (!s.instruction.empty()) e.set("instruction", vjson::Value::str(s.instruction));
        steps.push(e);
    }
    o.set("steps", steps);
    return o;
}

bool saveTemplate(const Template& t, const std::string& path, std::string& error) {
    if (path.empty()) {
        error = "this template has no file to save to";
        return false;
    }
    if (!vjson::writeFile(path, templateToJson(t), 2)) {
        error = "could not write " + path;
        return false;
    }
    return true;
}

std::vector<TemplateEntry> listTemplates(const std::string& dir) {
    std::vector<TemplateEntry> out;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return out;

    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const std::filesystem::path& p = entry.path();
        if (p.extension() != ".json") continue;

        Template t;
        std::string error;
        if (!loadTemplate(p.string(), t, error)) continue;

        TemplateEntry e;
        e.style = t.style;
        e.path = p.string();
        e.steps = static_cast<int>(t.steps.size());
        out.push_back(e);
    }

    std::sort(out.begin(), out.end(),
              [](const TemplateEntry& a, const TemplateEntry& b) { return a.style < b.style; });
    return out;
}

}  // namespace verimeasure
