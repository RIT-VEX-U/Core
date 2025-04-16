#include "core/device/vdb/types.hpp"
namespace VDP {
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
    // Name and type already read, only need to read number of fields before child
    // data shows up
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
void Record::setFields(std::vector<PartPtr> fs) { fields = std::move(fs); }
/**
 * sets the values of each Part the Record contains
 */
void Record::fetch() {
    for (auto &field : fields) {
        field->fetch();
    }
}
void Record::receive() {
    for (auto &field : fields) {
        field->receive();
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

        f->pprint(ss, indent + 1);
        ss << '\n';
    }
    add_indents(ss, indent);
    ss << "}\n";
}
/**
 * changes a stringstream to be formatted as
 * name: record[field size]{
 *  data pprint_dataed inside record
 * }
 * @param ss the stringstream to change
 * @param indent the amount of indents to use
 */
void Record::pprint_data(std::stringstream &ss, size_t indent) const {
    add_indents(ss, indent);
    ss << name << ": record[" << fields.size() << "]{\n";
    for (const auto &f : fields) {

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
/**
 * used to assign the string new data, runs the fetch function
 */
void String::fetch() { value = fetcher(); }
/**
 * function to run when receiving to this part
 */
void String::receive() {}
/**
 * sets the string part's value to the string given
 * @param new_value the string to set the value to
 */
void String::setValue(std::string new_value) { value = std::move(new_value); }
/**
 * sets the string part's value to the string read by a packet reader
 * @param reader the part reader to get
 */
void String::read_data_from_message(PacketReader &reader) { value = reader.get_string(); }
/**
 * changes a stringstream to be formatted as
 * name: string
 * @param ss the stringstream to change
 * @param indent the amount of indents to use
 */
void String::pprint(std::stringstream &ss, size_t indent) const {
    add_indents(ss, indent);
    ss << name << ": string";
}
/**
 * changes a stringstream to be formatted as
 * name: value
 * @param ss the stringstream to change
 * @param indent the amount of indents to use
 */
void String::pprint_data(std::stringstream &ss, size_t indent) const {
    add_indents(ss, indent);
    ss << name << ":\t" << value;
}
/**
 * writes the schematic for the string to a packet
 * @param sofar the packet writer to write with
 */
void String::write_schema(PacketWriter &sofar) const {
    sofar.write_type(Type::String); // Type
    sofar.write_string(name);       // Name
}
/**
 * writes the strings data to a packet
 * @param sofar the packet writer to write with
 */
void String::write_message(PacketWriter &sofar) const { sofar.write_string(value); }

} // namespace VDP
