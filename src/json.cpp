#include "verimeasure/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace vjson {

Value Value::null() { return Value(); }

Value Value::boolean(bool v) {
    Value x;
    x.type = Type::Bool;
    x.b = v;
    return x;
}

Value Value::integer(long long v) {
    Value x;
    x.type = Type::Int;
    x.i = v;
    return x;
}

Value Value::real(double v) {
    Value x;
    x.type = Type::Real;
    x.d = v;
    return x;
}

Value Value::str(const std::string& v) {
    Value x;
    x.type = Type::Str;
    x.s = v;
    return x;
}

Value Value::array() {
    Value x;
    x.type = Type::Array;
    return x;
}

Value Value::object() {
    Value x;
    x.type = Type::Object;
    return x;
}

void Value::set(const std::string& key, const Value& v) {
    if (type != Type::Object) {
        type = Type::Object;
        keys.clear();
        items.clear();
    }
    for (std::size_t k = 0; k < keys.size(); ++k) {
        if (keys[k] == key) {
            items[k] = v;
            return;
        }
    }
    keys.push_back(key);
    items.push_back(v);
}

const Value* Value::find(const std::string& key) const {
    if (type != Type::Object) return nullptr;
    for (std::size_t k = 0; k < keys.size(); ++k) {
        if (keys[k] == key) return &items[k];
    }
    return nullptr;
}

double Value::asNumber(double fallback) const {
    if (type == Type::Int) return static_cast<double>(i);
    if (type == Type::Real) return d;
    if (type == Type::Bool) return b ? 1.0 : 0.0;
    return fallback;
}

double Value::getNumber(const std::string& key, double fallback) const {
    const Value* v = find(key);
    return v ? v->asNumber(fallback) : fallback;
}

long long Value::getInt(const std::string& key, long long fallback) const {
    const Value* v = find(key);
    if (!v) return fallback;
    if (v->type == Type::Int) return v->i;
    if (v->type == Type::Real) return static_cast<long long>(v->d);
    return fallback;
}

bool Value::getBool(const std::string& key, bool fallback) const {
    const Value* v = find(key);
    if (!v) return fallback;
    if (v->type == Type::Bool) return v->b;
    if (v->type == Type::Int) return v->i != 0;
    return fallback;
}

std::string Value::getString(const std::string& key, const std::string& fallback) const {
    const Value* v = find(key);
    if (!v || v->type != Type::Str) return fallback;
    return v->s;
}

std::string formatDouble(double v) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity";

    char buf[64];
    for (int p = 1; p <= 17; ++p) {
        std::snprintf(buf, sizeof(buf), "%.*g", p, v);
        if (std::strtod(buf, nullptr) == v) break;
    }
    std::string out(buf);
    // A whole number must keep a decimal point, otherwise a float read back
    // becomes an int and the schema quietly changes type between runs.
    if (out.find('.') == std::string::npos && out.find('e') == std::string::npos &&
        out.find('E') == std::string::npos && out.find("inf") == std::string::npos &&
        out.find("nan") == std::string::npos) {
        out += ".0";
    }
    return out;
}

std::string formatFixed(double v, int decimals) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity";
    if (decimals < 1) decimals = 1;
    if (decimals > 9) decimals = 9;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    std::string out(buf);

    // Trim trailing zeros, but never past the first decimal place: a bench
    // display reads better when every value carries the same shape.
    const std::size_t dot = out.find('.');
    if (dot != std::string::npos) {
        std::size_t end = out.size();
        while (end > dot + 2 && out[end - 1] == '0') --end;
        out.erase(end);
    }
    // "-0.0" is an artefact of the sign bit, not a measurement.
    if (out == "-0.0") out = "0.0";
    return out;
}

static void escapeTo(std::string& out, const std::string& s) {
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    out += '"';
}

static void dumpTo(std::string& out, const Value& v, int indent, int depth) {
    const bool pretty = indent >= 0;
    const std::string pad = pretty ? std::string(indent * (depth + 1), ' ') : std::string();
    const std::string padEnd = pretty ? std::string(indent * depth, ' ') : std::string();

    switch (v.type) {
        case Value::Type::Null: out += "null"; break;
        case Value::Type::Bool: out += v.b ? "true" : "false"; break;
        case Value::Type::Int: out += std::to_string(v.i); break;
        case Value::Type::Real: out += formatDouble(v.d); break;
        case Value::Type::Str: escapeTo(out, v.s); break;
        case Value::Type::Array: {
            if (v.items.empty()) { out += "[]"; break; }
            out += '[';
            for (std::size_t k = 0; k < v.items.size(); ++k) {
                if (k) out += ',';
                if (pretty) { out += '\n'; out += pad; }
                dumpTo(out, v.items[k], indent, depth + 1);
            }
            if (pretty) { out += '\n'; out += padEnd; }
            out += ']';
            break;
        }
        case Value::Type::Object: {
            if (v.items.empty()) { out += "{}"; break; }
            out += '{';
            for (std::size_t k = 0; k < v.items.size(); ++k) {
                if (k) out += ',';
                if (pretty) { out += '\n'; out += pad; }
                escapeTo(out, v.keys[k]);
                out += pretty ? ": " : ":";
                dumpTo(out, v.items[k], indent, depth + 1);
            }
            if (pretty) { out += '\n'; out += padEnd; }
            out += '}';
            break;
        }
    }
}

std::string Value::dump(int indent) const {
    std::string out;
    dumpTo(out, *this, indent, 0);
    return out;
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

namespace {

struct Parser {
    const std::string& text;
    std::size_t pos = 0;
    std::string error;

    explicit Parser(const std::string& t) : text(t) {}

    void skip() {
        while (pos < text.size()) {
            char c = text[pos];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos;
            } else {
                break;
            }
        }
    }

    bool fail(const std::string& msg) {
        if (error.empty()) error = msg + " at offset " + std::to_string(pos);
        return false;
    }

    bool parseValue(Value& out);

    bool parseString(std::string& out) {
        if (pos >= text.size() || text[pos] != '"') return fail("expected string");
        ++pos;
        out.clear();
        while (pos < text.size()) {
            char c = text[pos++];
            if (c == '"') return true;
            if (c != '\\') { out += c; continue; }
            if (pos >= text.size()) return fail("truncated escape");
            char e = text[pos++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    if (pos + 4 > text.size()) return fail("truncated \\u escape");
                    unsigned cp = static_cast<unsigned>(
                        std::strtoul(text.substr(pos, 4).c_str(), nullptr, 16));
                    pos += 4;
                    // UTF-8 encode. Surrogate pairs are not expected in these
                    // files, so a lone surrogate is written through as-is.
                    if (cp < 0x80) {
                        out += static_cast<char>(cp);
                    } else if (cp < 0x800) {
                        out += static_cast<char>(0xC0 | (cp >> 6));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (cp >> 12));
                        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: return fail("unknown escape");
            }
        }
        return fail("unterminated string");
    }
};

bool Parser::parseValue(Value& out) {
    skip();
    if (pos >= text.size()) return fail("unexpected end of input");
    char c = text[pos];

    if (c == '{') {
        ++pos;
        out = Value::object();
        skip();
        if (pos < text.size() && text[pos] == '}') { ++pos; return true; }
        while (true) {
            skip();
            std::string key;
            if (!parseString(key)) return false;
            skip();
            if (pos >= text.size() || text[pos] != ':') return fail("expected ':'");
            ++pos;
            Value child;
            if (!parseValue(child)) return false;
            out.keys.push_back(key);
            out.items.push_back(child);
            skip();
            if (pos < text.size() && text[pos] == ',') { ++pos; continue; }
            if (pos < text.size() && text[pos] == '}') { ++pos; return true; }
            return fail("expected ',' or '}'");
        }
    }

    if (c == '[') {
        ++pos;
        out = Value::array();
        skip();
        if (pos < text.size() && text[pos] == ']') { ++pos; return true; }
        while (true) {
            Value child;
            if (!parseValue(child)) return false;
            out.items.push_back(child);
            skip();
            if (pos < text.size() && text[pos] == ',') { ++pos; continue; }
            if (pos < text.size() && text[pos] == ']') { ++pos; return true; }
            return fail("expected ',' or ']'");
        }
    }

    if (c == '"') {
        std::string s;
        if (!parseString(s)) return false;
        out = Value::str(s);
        return true;
    }

    if (text.compare(pos, 4, "true") == 0) { pos += 4; out = Value::boolean(true); return true; }
    if (text.compare(pos, 5, "false") == 0) { pos += 5; out = Value::boolean(false); return true; }
    if (text.compare(pos, 4, "null") == 0) { pos += 4; out = Value::null(); return true; }
    if (text.compare(pos, 8, "Infinity") == 0) { pos += 8; out = Value::real(HUGE_VAL); return true; }
    if (text.compare(pos, 9, "-Infinity") == 0) { pos += 9; out = Value::real(-HUGE_VAL); return true; }
    if (text.compare(pos, 3, "NaN") == 0) { pos += 3; out = Value::real(std::nan("")); return true; }

    // Number.
    std::size_t start = pos;
    bool isReal = false;
    if (pos < text.size() && (text[pos] == '-' || text[pos] == '+')) ++pos;
    while (pos < text.size()) {
        char n = text[pos];
        if (n >= '0' && n <= '9') { ++pos; continue; }
        if (n == '.' || n == 'e' || n == 'E' || n == '+' || n == '-') {
            if (n == '.' || n == 'e' || n == 'E') isReal = true;
            ++pos;
            continue;
        }
        break;
    }
    if (pos == start) return fail("unexpected character");
    std::string num = text.substr(start, pos - start);
    if (isReal) {
        out = Value::real(std::strtod(num.c_str(), nullptr));
    } else {
        out = Value::integer(std::strtoll(num.c_str(), nullptr, 10));
    }
    return true;
}

}  // namespace

bool parse(const std::string& text, Value& out, std::string& error) {
    // Skip a UTF-8 byte order mark. Every Windows editor an operator is likely
    // to open a template in writes one, PowerShell's redirect included, and
    // "unexpected character at offset 0" on a file that looks perfectly correct
    // is a miserable thing to debug on a factory floor.
    std::size_t start = 0;
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        start = 3;
    }
    const std::string body = start ? text.substr(start) : text;

    Parser p(body);
    if (!p.parseValue(out)) {
        error = p.error;
        return false;
    }
    p.skip();
    if (p.pos != body.size()) {
        error = "trailing content at offset " + std::to_string(p.pos + start);
        return false;
    }
    return true;
}

bool parseFile(const std::string& path, Value& out, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "could not open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse(ss.str(), out, error);
}

bool writeFile(const std::string& path, const Value& value, int indent) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << value.dump(indent);
    return static_cast<bool>(out);
}

}  // namespace vjson
