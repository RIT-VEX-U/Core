#include "../core/include/utils/serializer.h"
#include "stdlib.h"
#include "vex.h"

// ===================== Helper Functions for converting to and from bytes =====================
// Specialize these if you have non trivial types you need to save I suppose

// Convert a type to bytes to serialize

/// @brief Convert type to bytes. Overload this for non integer types
/// @param value value to convert
template <typename T>
std::vector<char> to_bytes(T value)
{
    // copy bytes of data into vector
    std::vector<char> value_bytes(sizeof(T));
    std::copy(static_cast<const char *>(static_cast<const void *>(&value)),
              static_cast<const char *>(static_cast<const void *>(&value)) + sizeof(value),
              value_bytes.begin());
    return value_bytes;
}

template <>
std::vector<char> to_bytes<std::string>(std::string str)
{
    std::vector<char> value_bytes(str.size() + 1);
    std::copy(str.begin(), str.end(), value_bytes.begin());
    value_bytes[str.size()] = 0x0;
    return value_bytes;
}

/// @brief Convert bytes to a type
/// @param gets data from arbitrary bytes. Overload this for non integer types
template <typename T>
T from_bytes(std::vector<char>::const_iterator &position)
{
    T value;
    std::copy(position, position + sizeof(T), static_cast<char *>(static_cast<void *>(&value)));
    position = position + sizeof(T);
    return value;
}

template <>
std::string from_bytes(std::vector<char>::const_iterator &position)
{
    auto pos = position;
    while (*pos != 0x0)
    {
        ++pos;
    }
    std::string s(position, pos);
    position = pos + 1;
    return s;
}

/// @brief Replaces funny characters in names so they don't mess with serialization specifiers
std::string sanitize_name(std::string s)
{
    std::replace(s.begin(), s.end(), serialization_separator, '-');
    return s;
}

// Protocol
/*
 * Null terminated name immediatly followed by the bytes of the value
 * +----Ints-----+
 * |name\0 bytes |
 * |   [sep]     |
 * +---Bools-----+
 * |name\0 byte  |
 * |   [sep]     |
 * +---Doubles --+
 * |name\0 bytes |
 * |   [sep]     |
 * +---Strings --+
 * |name\0 str\0 |
 * |   [sep]     |
 * +-------------+
 */

/// @brief Adds data to a file (represented by array of bytes) as specified by the format above
/// @param data the bytes to add to
/// @param map the values and names we are talking about
template <typename value_type>
static void add_data(std::vector<char> &data, const std::map<std::string, value_type> &map)
{
    for (const auto &pair : map)
    {
        const std::string &name = pair.first;
        const value_type value = pair.second;

        data.insert(data.end(), name.cbegin(), name.cend());
        data.insert(data.end(), 0x0);

        auto bytes = to_bytes<value_type>(value);
        data.insert(data.end(), bytes.cbegin(), bytes.cend());
    }
    data.push_back(serialization_separator);
}

// reads data of a certain type from a file
template <typename value_type>
static std::vector<char>::const_iterator read_data(const std::vector<char> &data,
                                                   std::vector<char>::const_iterator begin,
                                                   std::map<std::string, value_type> &map)
{
    std::vector<char>::const_iterator pos = begin;

    while (*pos != serialization_separator)
    {
        auto name_start = pos;
        // read name
        std::string name = from_bytes<std::string>(name_start);
        pos += name.size() + 1;
        // read value
        value_type value = from_bytes<value_type>(pos);
        printf("Read Name: %s\n", name.c_str());
        map.insert({name, value});
    }

    return pos + 1;
}

void Serializer::set_int(const std::string &name, int i)
{
    ints[sanitize_name(name)] = i;
    if (flush_always)
    {
        save_to_disk();
    }
}
void Serializer::set_bool(const std::string &name, bool b)
{
    bools[sanitize_name(name)] = b;
    if (flush_always)
    {
        save_to_disk();
    }
}
void Serializer::set_double(const std::string &name, double d)
{
    doubles[sanitize_name(name)] = d;
    if (flush_always)
    {
        save_to_disk();
    }
}
void Serializer::set_string(const std::string &name, std::string str)
{
    strings[sanitize_name(name)] = str;
    if (flush_always)
    {
        save_to_disk();
    }
}

int Serializer::int_or(const std::string &name, int otherwise)
{
    if (ints.count(name))
    {
        return ints.at(name);
    }
    set_int(name, otherwise);
    return otherwise;
}
bool Serializer::bool_or(const std::string &name, bool otherwise)
{
    if (bools.count(name))
    {
        return bools.at(name);
    }
    set_bool(name, otherwise);
    return otherwise;
}
double Serializer::double_or(const std::string &name, double otherwise)
{
    if (doubles.count(name))
    {
        return doubles.at(name);
    }
    set_double(name, otherwise);
    return otherwise;
}
std::string Serializer::string_or(const std::string &name, std::string otherwise)
{
    if (strings.count(name))
    {
        return strings.at(name);
    }
    set_string(name, otherwise);
    return otherwise;
}

/// @brief forms data bytes then saves to filename this was openned with
void Serializer::save_to_disk() const
{
    std::vector<char> data = {};
    add_data<int>(data, ints);
    add_data<bool>(data, bools);
    add_data<double>(data, doubles);
    add_data<std::string>(data, strings);

    fflush(stdout);

    vex::brain::sdcard sd;
    int32_t written = sd.savefile(filename.c_str(), (unsigned char *)&data[0], data.size());
    if (written != data.size())
    {
        printf("!! Error writing to `%s`!!", filename.c_str());
        return;
    }
}

/// @brief reads types from file data
bool Serializer::read_from_disk()
{
    vex::brain::sdcard sd;
    if (!sd.isInserted())
    {
        printf("!! Trying to Serialize to No SD Card !!\n");
        return false;
    }
    int size = sd.size(filename.c_str());

    if (filename == ""){
        printf("!! Can't write to empty filename!!\n");
        return false;
    }

    if (!sd.exists(filename.c_str()))
    {
        printf("!!f Trying to Serialize to a file that doesnt exist: Creating %s !!\n", filename.c_str());
        save_to_disk();
        return false;
    }

    std::vector<char> data(size);

    int readsize = sd.loadfile(filename.c_str(), (unsigned char *)&data[0], size);
    if (size != readsize)
    {
        printf("!! Error reading from file !!");;
        return false;
    }

    auto bool_start = read_data<int>(data, data.cbegin(), ints);
    auto doubles_start = read_data<bool>(data, bool_start, bools);
    auto strings_start = read_data<double>(data, doubles_start, doubles);
    auto file_end = read_data<std::string>(data, strings_start, strings);
    (void)file_end;

    return true;
}
