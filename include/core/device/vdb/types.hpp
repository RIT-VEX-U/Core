#pragma once
#include "core/device/vdb/protocol.hpp"
#include <string>
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
    void set_fields(std::vector<PartPtr> fields);

    std::vector<PartPtr> get_fields() const;

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

    PartPtr clone() override;

    void Visit(Visitor *);

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
    void set_value(std::string new_value);

    /**
     * @return the currently stored string
     */
    std::string get_value();

    PartPtr clone() override;
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

    void Visit(Visitor *);

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
    void set_value(NumberType val) { this->value = val; }
    /**
     * @return the currently stored number value
     */
    NumberType get_value() { return value; }
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
    FetchFunc fetcher;
    NumberType value = (NumberType)0;    
};

class Float : public Number<float, Type::Float> {
public:
  using NumT = Number<float, Type::Float>;
  Float(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Double : public Number<double, Type::Double> {
public:
  using NumT = Number<double, Type::Double>;
  Double(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};

class Uint8 : public Number<uint8_t, Type::Uint8> {
public:
  using NumT = Number<uint8_t, Type::Uint8>;
  Uint8(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Uint16 : public Number<uint16_t, Type::Uint16> {
public:
  using NumT = Number<uint16_t, Type::Uint16>;
  Uint16(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Uint32 : public Number<uint32_t, Type::Uint32> {
public:
  using NumT = Number<uint32_t, Type::Uint32>;
  Uint32(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Uint64 : public Number<uint64_t, Type::Uint64> {
public:
  using NumT = Number<uint64_t, Type::Uint64>;
  Uint64(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};

class Int8 : public Number<int8_t, Type::Int8> {
public:
  using NumT = Number<int8_t, Type::Int8>;
  Int8(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Int16 : public Number<int16_t, Type::Int16> {
public:
  using NumT = Number<int16_t, Type::Int16>;
  Int16(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
class Int32 : public Number<int32_t, Type::Int32> {
public:
  using NumT = Number<int32_t, Type::Int32>;
  Int32(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};

class Int64 : public Number<int64_t, Type::Int64> {
public:
  using NumT = Number<int64_t, Type::Int64>;
  Int64(
      std::string name,
      NumT::FetchFunc func = []() { return (NumT::NumberType)0; });
  void Visit(Visitor *);
  PartPtr clone() override;
};
/**
 * A class for broadly visiting a part and doing some action based on the type of part
 */
class Visitor {
public:
  virtual ~Visitor() {}
  
  virtual void VisitRecord(Record *) = 0;

  virtual void VisitString(String *) = 0;

  virtual void VisitFloat(Float *) = 0;
  virtual void VisitDouble(Double *) = 0;

  virtual void VisitUint8(Uint8 *) = 0;
  virtual void VisitUint16(Uint16 *) = 0;
  virtual void VisitUint32(Uint32 *) = 0;
  virtual void VisitUint64(Uint64 *) = 0;

  virtual void VisitInt8(Int8 *) = 0;
  virtual void VisitInt16(Int16 *) = 0;
  virtual void VisitInt32(Int32 *) = 0;
  virtual void VisitInt64(Int64 *) = 0;
};
/**
 * A class for broadly visiting a part and doing some action based on the upcast type of the part
 */
class UpcastNumbersVisitor : public Visitor {
public:
  virtual void VisitAnyFloat(const std::string &name, double value,
                             const Part *) = 0;
  virtual void VisitAnyInt(const std::string &name, int64_t value,
                           const Part *) = 0;
  virtual void VisitAnyUint(const std::string &name, uint64_t value,
                            const Part *) = 0;

  // Implemented to call Visitor::VisitAnyFloat
  void VisitFloat(Float *) override;
  void VisitDouble(Double *) override;

  // Implemented to call Visitor::VisitAnyUint
  void VisitUint8(Uint8 *) override;
  void VisitUint16(Uint16 *) override;
  void VisitUint32(Uint32 *) override;
  void VisitUint64(Uint64 *) override;

  // Implemented to call Visitor::VisitAnyInt
  void VisitInt8(Int8 *) override;
  void VisitInt16(Int16 *) override;
  void VisitInt32(Int32 *) override;
  void VisitInt64(Int64 *) override;
};

} // namespace VDP
