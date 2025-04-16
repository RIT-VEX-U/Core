#pragma once
#include "core/device/vdb/protocol.hpp"

namespace VDP {
/**
 * Defines a Part that contains another Part
 * essentially an array of parts that is formatted so that it can be sent to the debug board
 */
class Record : public Part {
    friend PacketReader;
    friend PacketWriter;

  public:
    using SizeT = uint32_t;
    /**
     * Creates a Record with just a name
     * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
     * @param name the name for the part
     */
    explicit Record(std::string name);
    /**
     * Creates a Record with a name that contains the Parts inside a vector of Parts
     * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
     * @param name the name for the part
     * @param parts the vector of Parts for the record to hold
     */
    Record(std::string name, const std::vector<Part *> &parts);
    /**
     * Creates a Record with a name that contains the Parts inside a vector of Part Pointers
     * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
     * @param name the name for the part
     * @param parts the vector of Part Pointers for the record to hold
     */
    Record(std::string name, std::vector<PartPtr> parts);
    /**
     * Creates a record with a name based off of a packet read by a PacketReader
     * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
     * @param name
     * @param reader
     */
    Record(std::string name, PacketReader &reader);
    /**
     * sets the Record to contain Parts from a part Pointer
     * @param fs the vector of Part Pointers for the record to hold
     */
    void setFields(std::vector<PartPtr> fields);

    /**
     * sets the values of each Part the Record contains
     */
    void fetch() override;

    void response() override;
    /**
     * writes a message to the packet containing part record
     * @param sofar the PacketWriter to write with
     */
    void read_data_from_message(PacketReader &reader) override;

  protected:
    // Encode the schema itself for transmission on the wire
    void write_schema(PacketWriter &sofar) const override;
    // Encode the data currently held according to schema for transmission on the
    // wire
    void write_message(PacketWriter &sofar) const override;

  private:
    void pprint(std::stringstream &ss, size_t indent) const override;
    void pprint_data(std::stringstream &ss, size_t indent) const override;

    std::vector<PartPtr> fields;
};
/**
 * A string type conveyed as a part
 */
class String : public Part {
    friend PacketReader;
    friend PacketWriter;

  public:
    using FetchFunc = std::function<std::string()>;
    /**
     * creates a string type conveyed as a part with a name and a fetcher
     * @param name name of the string to have
     * @param fetcher the fetch function to use when running fetch()
     */
    explicit String(std::string name, FetchFunc fetcher = []() { return "no value"; });
    /**
     * function to run when fetching this part, runs the fetch function
     */
    void fetch() override;

    /**
     * function to run when receiving to this part
     */
    void response() override;

    /**
     * sets the string part's value to the string given
     * @param new_value the string to set the value to
     */
    void setValue(std::string new_value);
    /**
     * sets the string part's value to the string read by a packet reader
     * @param reader the packet reader to get the string from
     */
    void read_data_from_message(PacketReader &reader) override;
    /**
     * changes a stringstream to be formatted as
     * name: string
     * @param ss the stringstream to change
     * @param indent the amount of indents to use
     */
    void pprint(std::stringstream &ss, size_t indent) const override;
    /**
     * changes a stringstream to be formatted as
     * name: value
     * @param ss the stringstream to change
     * @param indent the amount of indents to use
     */
    void pprint_data(std::stringstream &ss, size_t indent) const override;

  protected:
    void write_schema(PacketWriter &sofar) const override;
    void write_message(PacketWriter &sofar) const override;

  private:
    FetchFunc fetcher;
    std::string value;
};

// Template to reduce boiler plate for Schema wrappers for simple types
// Fixed size, numeric types  such as uin8_t, uint32, float, double
/**
 * A number conveyed as a part
 */
template <typename NumT, Type schemaType> class Number : public Part {
    friend PacketReader;
    friend PacketWriter;

  public:
    using NumberType = NumT;
    static constexpr Type SchemaType = schemaType;

    // Checks to make sure this isn't misused
    static_assert(
      std::is_floating_point<NumberType>::value || std::is_integral<NumberType>::value,
      "Number type this is instantiated with must be floating point "
      "or integral"
    );
    /**
     * Function to run when fetching this number
     */
    using FetchFunc = std::function<NumberType()>; /**
                                                    * creates a number with a name and fetcher
                                                    * @param field name for the number part
                                                    * @param fetcher the function to run when fetching this number
                                                    */
    explicit Number(
      std::string field_name, FetchFunc fetcher = []() { return (NumberType)0; }
    )
        : Part(field_name), fetcher(fetcher) {}
    /**
     * sets the value of the number stored to the value returned by its fetcher
     */
    void fetch() override { value = fetcher(); }
    /**
     * sets the value of the number stored
     * @param val the value to store
     */
    void setValue(NumberType val) { this->value = val; }
    /**
     * @return the currently stored number value
     */
    NumberType getValue() { return value; }
    /**
     * prints the Number with the format "[indent]name: schema_string"
     * @param ss the stream of strings to print to
     * @param indent the amount of indents to use
     */
    void pprint(std::stringstream &ss, size_t indent) const override {
        add_indents(ss, indent);
        ss << name << ":\t" << to_string(SchemaType);
    }
    /**
     * prints the data the number holds with the format "[indent]name: value"
     * @param ss the stream of strings to print to
     * @param indent the amount of indents to use
     */
    void pprint_data(std::stringstream &ss, size_t indent) const override {
        add_indents(ss, indent);
        ss << name << ":\t";
        if (sizeof(NumberType) == 1) {
            ss << (int)value; // Otherwise, stringstream interprets uint8 as char and
                              // prints a char
        } else {
            ss << value;
        }
    }
    /**
     * sets the value of the number stored to the value read by a PacketReader
     * @param reader the packet reader to get the number from
     */
    void read_data_from_message(PacketReader &reader) override { value = reader.get_number<NumberType>(); }

  protected:
    /**
     * writes the number's schematic to a packet
     * @param sofar the packet writer to write with
     */
    void write_schema(PacketWriter &sofar) const override {
        sofar.write_type(SchemaType); // Type
        sofar.write_string(name);     // Name
    }
    /**
     * writes the number's data to a packet
     * @param sofar the packet writer to write with
     */
    void write_message(PacketWriter &sofar) const override { sofar.write_number<NumberType>(value); }

  private:
    FetchFunc fetcher;
    NumberType value = (NumberType)0;
};

using Float = Number<float, Type::Float>;
using Double = Number<double, Type::Double>;

using Uint8 = Number<uint8_t, Type::Uint8>;
using Uint16 = Number<uint16_t, Type::Uint16>;
using Uint32 = Number<uint32_t, Type::Uint32>;
using Uint64 = Number<uint64_t, Type::Uint64>;

using Int8 = Number<int8_t, Type::Int8>;
using Int16 = Number<int16_t, Type::Int16>;
using Int32 = Number<int32_t, Type::Int32>;
using Int64 = Number<int64_t, Type::Int64>;

} // namespace VDP
