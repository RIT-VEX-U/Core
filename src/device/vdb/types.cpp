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
void Record::set_fields(std::vector<PartPtr> fs) { fields = std::move(fs); }

std::vector<PartPtr> Record::get_fields() const { return fields; }

PartPtr Record::clone(){
    std::shared_ptr<Record> cloned_record = std::make_shared<Record>(this->name);
    std::vector<PartPtr> cloned_fields;
    for(auto field : this->fields){
        cloned_fields.push_back(field->clone());
    }
    cloned_record->set_fields(cloned_fields);
    return cloned_record;
}
/**
 * sets the values of each Part the Record contains
 */
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
void String::response() {}
/**
 * sets the string part's value to the string given
 * @param new_value the string to set the value to
 */
void String::set_value(std::string new_value) { value = std::move(new_value); }
/**
 * @return the currently stored string
 */
std::string String::get_value() { return value; }

PartPtr String::clone() {
    std::shared_ptr<String> cloned_string = std::make_shared<String>(this->name);
    cloned_string->set_value(this->value);
    return cloned_string;
};
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

Float::Float(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Double::Double(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Uint8::Uint8(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Uint16::Uint16(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Uint32::Uint32(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Uint64::Uint64(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Int8::Int8(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Int16::Int16(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Int32::Int32(std::string name, NumT::FetchFunc func) : NumT(name, func) {}
Int64::Int64(std::string name, NumT::FetchFunc func) : NumT(name, func) {}

void Record::Visit(Visitor *v) { v->VisitRecord(this); }
void String::Visit(Visitor *v) { v->VisitString(this); }

void Float::Visit(Visitor *v) { v->VisitFloat(this); }
void Double::Visit(Visitor *v) { v->VisitDouble(this); }

void Uint8::Visit(Visitor *v) { v->VisitUint8(this); }
void Uint16::Visit(Visitor *v) { v->VisitUint16(this); }
void Uint32::Visit(Visitor *v) { v->VisitUint32(this); }
void Uint64::Visit(Visitor *v) { v->VisitUint64(this); }

void Int8::Visit(Visitor *v) { v->VisitInt8(this); }
void Int16::Visit(Visitor *v) { v->VisitInt16(this); }
void Int32::Visit(Visitor *v) { v->VisitInt32(this); }
void Int64::Visit(Visitor *v) { v->VisitInt64(this); }

void UpcastNumbersVisitor::VisitFloat(Float *f) {
  VisitAnyFloat(f->get_name(), f->get_value(), f);
}
void UpcastNumbersVisitor::VisitDouble(Double *f) {
  VisitAnyFloat(f->get_name(), f->get_value(), f);
}
void UpcastNumbersVisitor::VisitUint8(Uint8 *f) {
  VisitAnyUint(f->get_name(), (uint64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitUint16(Uint16 *f) {
  VisitAnyUint(f->get_name(), (uint64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitUint32(Uint32 *f) {
  VisitAnyUint(f->get_name(), (uint64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitUint64(Uint64 *f) {
  VisitAnyUint(f->get_name(), (uint64_t)f->get_value(), f);
}

void UpcastNumbersVisitor::VisitInt8(Int8 *f) {
  VisitAnyUint(f->get_name(), (int64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitInt16(Int16 *f) {
  VisitAnyUint(f->get_name(), (int64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitInt32(Int32 *f) {
  VisitAnyUint(f->get_name(), (int64_t)f->get_value(), f);
}
void UpcastNumbersVisitor::VisitInt64(Int64 *f) {
  VisitAnyUint(f->get_name(), (int64_t)f->get_value(), f);
}

PartPtr Float::clone(){
    std::shared_ptr<Float> clone = std::make_shared<Float>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Double::clone(){
    std::shared_ptr<Double> clone = std::make_shared<Double>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Uint8::clone(){
    std::shared_ptr<Uint8> clone = std::make_shared<Uint8>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Uint16::clone(){
    std::shared_ptr<Uint16> clone = std::make_shared<Uint16>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Uint32::clone(){
    std::shared_ptr<Uint32> clone = std::make_shared<Uint32>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Uint64::clone(){
    std::shared_ptr<Uint64> clone = std::make_shared<Uint64>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Int8::clone(){
    std::shared_ptr<Int8> clone = std::make_shared<Int8>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Int16::clone(){
    std::shared_ptr<Int16> clone = std::make_shared<Int16>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Int32::clone(){
    std::shared_ptr<Int32> clone = std::make_shared<Int32>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

PartPtr Int64::clone(){
    std::shared_ptr<Int64> clone = std::make_shared<Int64>(this->name, this->fetcher);
    clone->set_value(this->value);
    return clone;
}

} // namespace VDP
