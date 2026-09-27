#include "core/device/vdb/types.hpp"

namespace VDP {
<<<<<<< HEAD
std::string to_string(TypeId t) {
  switch (t) {
    case TypeId::Record:
      return "record";
    case TypeId::Boolean:
      return "boolean";
    case TypeId::String:
      return "string";
=======
/**
 * Creates a Record with just a name
 * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
 * @param name the name for the part
 */
Record::Record(std::string name) : Part(std::move(name)), fields({}) {}
/**
 * Creates a Record with a name that contains the Parts inside a vector of Parts
 * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
 * @param name the name for the part
 * @param parts the vector of Parts for the record to hold
 */
Record::Record(std::string name, const std::vector<Part *> &parts) : Part(std::move(name)), fields() {
    fields.reserve(parts.size());
    for (Part *f : parts) {
        fields.emplace_back(f);
    }
}
/**
 * Creates a Record with a name that contains the Parts inside a vector of Part Pointers
 * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
 * @param name the name for the Record
 * @param parts the vector of Part Pointers for the record to hold
 */
Record::Record(std::string name, std::vector<PartPtr> parts) : Part(std::move(name)), fields(std::move(parts)) {}
/**
 * Creates a record with a name based off of a packet read by a PacketReader
 * a Record is essentially an array of parts that is formatted so that it can be sent to the debug board
 * @param name
 * @param reader
 */
Record::Record(std::string name, PacketReader &reader) : Part(std::move(name)), fields() {
    /* 
     * Name and type already read, only need to read number of fields before child
     * data shows up
     */
    const uint32_t size = reader.get_number<SizeT>();
    fields.reserve(size);
    for (size_t i = 0; i < size; i++) {
        fields.push_back(make_decoder(reader));
    }
}
/**
 * sets the Record to contain Parts from a part Pointer
 * @param fs the vector of Part Pointers for the record to hold
 */
void Record::set_fields(std::vector<PartPtr> fs) { fields = std::move(fs); }
>>>>>>> main

    case TypeId::Float:
      return "float";
    case TypeId::Double:
      return "double";

<<<<<<< HEAD
    case TypeId::Uint8:
      return "uint8";
    case TypeId::Uint16:
      return "uint16";
    case TypeId::Uint32:
      return "uint32";
    case TypeId::Uint64:
      return "uint64";
=======
PartPtr Record::clone(){
    std::shared_ptr<Record> cloned_record = std::make_shared<Record>(this->name);
    std::vector<PartPtr> cloned_fields;
    for(auto field : this->fields){
        cloned_fields.push_back(field->clone());
    }
    cloned_record->set_fields(cloned_fields);
    return cloned_record;
}
/// sets the values of each Part the Record contains
void Record::fetch() {
    for (auto &field : fields) {
        field->fetch();
    }
}
void Record::response() {
    for (auto &field : fields) {
        field->response();
    }
}
void Record::read_data_from_message(PacketReader &reader) {
    for (auto &f : fields) {
        f->read_data_from_message(reader);
    }
}
/**
 * writes the Record as the
 */
void Record::write_schema(PacketWriter &sofar) const {
    sofar.write_type(Type::Record);           // Type
    sofar.write_string(name);                 // Name
    sofar.write_number<SizeT>(fields.size()); // Number of fields
    for (const PartPtr &field : fields) {
        field->write_schema(sofar);
    }
}
/**
 * writes a message to the packet containing part record
 * @param sofar the PacketWriter to write with
 */
void Record::write_message(PacketWriter &sofar) const {
    for (auto &f : fields) {
        f->write_message(sofar);
    }
}
/**
 * changes a stringstream to be formatted as
 * name: record[field size]{
 *  data pprinted inside record
 * }
 * @param ss the stringstream to change
 * @param indent the amount of indents to use
 */
void Record::pprint(std::stringstream &ss, size_t indent) const {
    add_indents(ss, indent);
    ss << name << ": record[" << fields.size() << "]{\n";
    for (const auto &f : fields) {
>>>>>>> main

    case TypeId::Int8:
      return "int8";
    case TypeId::Int16:
      return "int16";
    case TypeId::Int32:
      return "int32";
    case TypeId::Int64:
      return "int64";

<<<<<<< HEAD
    case TypeId::Q3_4:
      return "Q3_4";
    case TypeId::Q4_4:
      return "Q4_4";
    case TypeId::Q7_1:
      return "Q7_1";
    case TypeId::Q1_7:
      return "Q1_7";
    case TypeId::Q6_2:
      return "Q6_2";
    case TypeId::Q2_6:
      return "Q2_6";
    case TypeId::Q7_8:
      return "Q7_8";
    case TypeId::Q8_8:
      return "Q8_8";
    case TypeId::Q15_1:
      return "Q15_1";
    case TypeId::Q1_15:
      return "Q1_15";
    case TypeId::Q9_6:
      return "Q9_6";
    case TypeId::Q10_6:
      return "Q10_6";
    case TypeId::Q12_12:
      return "Q12_12";
    case TypeId::Q16_8:
      return "Q16_8";
    case TypeId::Q8_16:
      return "Q8_16";
    case TypeId::Q15_16:
      return "Q15_16";
    case TypeId::Q16_16:
      return "Q16_16";
    case TypeId::Q24_8:
      return "Q24_8";
    case TypeId::Q8_24:
      return "Q8_24";
    case TypeId::Q31_32:
      return "Q31_32";
    case TypeId::Q32_32:
      return "Q32_32";
    default:
      return "unknown";
  }
=======
        f->pprint_data(ss, indent + 1);
        ss << '\n';
    }
    add_indents(ss, indent);
    ss << "}\n";
}
/**
 * creates a string type conveyed as a part with a name and a fetcher
 * @param name name of the string to have
 * @param fetcher the fetcher function to use when assigning it new data
 */
String::String(std::string field_name, std::function<std::string()> fetcher)
    : Part(std::move(field_name)), fetcher(std::move(fetcher)) {}

/// used to assign the string new data, runs the fetch function
void String::fetch() { value = fetcher(); }

/// function to run when receiving to this part
void String::response() {}
/**
 * sets the string part's value to the string given
 * @param new_value the string to set the value to
 */
void String::set_value(std::string new_value) { value = std::move(new_value); }

/// @return the currently stored string
std::string String::get_value() { return value; }
>>>>>>> main

  return "<<UNKNOWN TYPE>>";
}

TypeId parse_type(std::string str) {
  if (str == "record") return TypeId::Record;
  if (str == "string") return TypeId::String;

  if (str == "float") return TypeId::Float;
  if (str == "double") return TypeId::Double;

  if (str == "uint8") return TypeId::Uint8;
  if (str == "uint16") return TypeId::Uint16;
  if (str == "uint32") return TypeId::Uint32;
  if (str == "uint64") return TypeId::Uint64;

  if (str == "int8") return TypeId::Int8;
  if (str == "int16") return TypeId::Int16;
  if (str == "int32") return TypeId::Int32;
  if (str == "int64") return TypeId::Int64;

  if (str == "Q3_4") return TypeId::Q3_4;
  if (str == "Q4_4") return TypeId::Q4_4;
  if (str == "Q7_1") return TypeId::Q7_1;
  if (str == "Q1_7") return TypeId::Q1_7;
  if (str == "Q6_2") return TypeId::Q6_2;
  if (str == "Q2_6") return TypeId::Q2_6;
  if (str == "Q7_8") return TypeId::Q7_8;
  if (str == "Q8_8") return TypeId::Q8_8;
  if (str == "Q15_1") return TypeId::Q15_1;
  if (str == "Q1_15") return TypeId::Q1_15;
  if (str == "Q9_6") return TypeId::Q9_6;
  if (str == "Q10_6") return TypeId::Q10_6;
  if (str == "Q12_12") return TypeId::Q12_12;
  if (str == "Q16_8") return TypeId::Q16_8;
  if (str == "Q8_16") return TypeId::Q8_16;
  if (str == "Q15_16") return TypeId::Q15_16;
  if (str == "Q16_16") return TypeId::Q16_16;
  if (str == "Q24_8") return TypeId::Q24_8;
  if (str == "Q8_24") return TypeId::Q8_24;
  if (str == "Q31_32") return TypeId::Q31_32;
  if (str == "Q32_32") return TypeId::Q32_32;
  return TypeId::UNKNOWN;
}
}  // namespace VDP
