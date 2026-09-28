// Minimal ordered JSON reader/writer.
//
// The Python original leaned on the json module. Pulling in a third-party
// library for four small config files would be a poor trade on a machine that
// only has the Arena SDK environment installed, so this is self-contained.
// Object key order is preserved, and doubles are written with the shortest
// representation that round-trips, which is what Python's json.dump produces.

#pragma once

#include <string>
#include <vector>

namespace vjson {

class Value {
public:
    enum class Type { Null, Bool, Int, Real, Str, Array, Object };

    Type type = Type::Null;
    bool b = false;
    long long i = 0;
    double d = 0.0;
    std::string s;
    std::vector<Value> items;       // array elements, or object values
    std::vector<std::string> keys;  // object keys, parallel to items

    Value() = default;

    static Value null();
    static Value boolean(bool v);
    static Value integer(long long v);
    static Value real(double v);
    static Value str(const std::string& v);
    static Value array();
    static Value object();

    bool isNull() const { return type == Type::Null; }
    bool isObject() const { return type == Type::Object; }
    bool isArray() const { return type == Type::Array; }
    bool isNumber() const { return type == Type::Int || type == Type::Real; }

    // Object access. set() replaces in place if the key exists, otherwise
    // appends, so insertion order matches the Python dict it mirrors.
    void set(const std::string& key, const Value& v);
    const Value* find(const std::string& key) const;
    bool has(const std::string& key) const { return find(key) != nullptr; }

    // Array access.
    void push(const Value& v) { items.push_back(v); }
    std::size_t size() const { return items.size(); }
    const Value& at(std::size_t idx) const { return items[idx]; }

    // Convenience readers. Each returns the fallback when the key is absent or
    // the stored type cannot be converted.
    double getNumber(const std::string& key, double fallback = 0.0) const;
    long long getInt(const std::string& key, long long fallback = 0) const;
    bool getBool(const std::string& key, bool fallback = false) const;
    std::string getString(const std::string& key, const std::string& fallback = "") const;

    double asNumber(double fallback = 0.0) const;

    std::string dump(int indent = -1) const;
};

// Returns false and fills error on malformed input.
bool parse(const std::string& text, Value& out, std::string& error);
bool parseFile(const std::string& path, Value& out, std::string& error);
bool writeFile(const std::string& path, const Value& value, int indent = 2);

// Shortest round-tripping decimal form, with a trailing ".0" on whole numbers
// so a float never reads back as an int. This is for machines: it will happily
// write 470.0 as "4.7e+02", because that round-trips and is shorter.
std::string formatDouble(double v);

// The same number for a person to read: plain decimal, never exponent form,
// with trailing zeros trimmed but always at least one decimal place. Use this
// for anything that reaches an operator, a log line or a console. "4.7e+02 mm"
// on a bench display is not a measurement, it is a puzzle.
std::string formatFixed(double v, int decimals = 2);

}  // namespace vjson
