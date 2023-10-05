#pragma once
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <stdio.h>

/// @brief character that will be used to seperate values
const char serialization_separator = '$';
/// @brief max file size that the system can deal with
const std::size_t MAX_FILE_SIZE = 4096; 

/// @brief Serializes Arbitrary data to a file on the SD Card
class Serializer
{
private:
    bool flush_always;
    std::string filename;
    std::map<std::string, int> ints;
    std::map<std::string, bool> bools;
    std::map<std::string, double> doubles;
    std::map<std::string, std::string> strings;

    /// @brief loads Serializer state from disk
    bool read_from_disk();

public:
    /// @brief Save and close upon destruction (bc of vex, this doesnt always get called when the program ends. To be sure, call save_to_disk)
    ~Serializer()
    {
        save_to_disk();
        printf("Saving %s\n", filename.c_str());
        fflush(stdout);
    }

    /// @brief create a Serializer
    /// @param filename the file to read from. If filename does not exist we will create that file
    /// @param flush_always If true, after every write flush to a file. If false, you are responsible for calling save_to_disk
    explicit Serializer(const std::string &filename, bool flush_always = true) : flush_always(flush_always), filename(filename), ints({}), bools({}), doubles({}), strings({}) { read_from_disk(); }

    /// @brief saves current Serializer state to disk
    void save_to_disk() const;

    /// Setters - not saved until save_to_disk is called

    /// @brief sets an integer by the name of name to i. If flush_always == true, this will save to the sd card
    /// @param name name of integer
    /// @param i value of integer
    void set_int(const std::string &name, int i);

    /// @brief sets a bool by the name of name to b. If flush_always == true, this will save to the sd card
    /// @param name name of bool
    /// @param b value of bool
    void set_bool(const std::string &name, bool b);

    /// @brief sets a double by the name of name to d. If flush_always == true, this will save to the sd card
    /// @param name name of double
    /// @param d value of double
    void set_double(const std::string &name, double d);

    /// @brief sets a string by the name of name to s. If flush_always == true, this will save to the sd card
    /// @param name name of string
    /// @param i value of string
    void set_string(const std::string &name, std::string str);

    /// Getters
    /// Return value if it exists in the serializer

    /// @brief gets a value stored in the serializer. If not found, sets the value to otherwise
    /// @param name name of value
    /// @param otherwise value if the name is not specified
    /// @return the value if found or otherwise
    int int_or(const std::string &name, int otherwise);

    /// @brief gets a value stored in the serializer. If not, sets the value to otherwise
    /// @param name name of value
    /// @param otherwise value if the name is not specified
    /// @return the value if found or otherwise
    bool bool_or(const std::string &name, bool otherwise);

    /// @brief gets a value stored in the serializer. If not, sets the value to otherwise
    /// @param name name of value
    /// @param otherwise value if the name is not specified
    /// @return the value if found or otherwise
    double double_or(const std::string &name, double otherwise);

    /// @brief gets a value stored in the serializer. If not, sets the value to otherwise
    /// @param name name of value
    /// @param otherwise value if the name is not specified
    /// @return the value if found or otherwise
    std::string string_or(const std::string &name, std::string otherwise);
};
