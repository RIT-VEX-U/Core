#include "core/device/vdb/visitor.hpp"

ResponsePacketVisitor::ResponsePacketVisitor(VDP::PartPtr from_part) : from_part(from_part){}

void ResponsePacketVisitor::VisitRecord(VDP::Record *record) {
  //since we make each response based on a schema, they should have the exact same types
  VDP::Record *from_record = reinterpret_cast<VDP::Record*>(from_part.get());
  for (int i = 0; i < record->get_fields().size(); i++) {
    ResponsePacketVisitor new_RV(from_record->get_fields().at(i));
    record->get_fields().at(i)->Visit(&new_RV);
  }
}

void ResponsePacketVisitor::VisitString(VDP::String *str) {
  VDP::String *from_str = reinterpret_cast<VDP::String*>(from_part.get());
  if(from_str->get_value() != (std::string)NULL){
    printf("found string\n");
    str->set_value(from_str->get_value());
  }
}
void ResponsePacketVisitor::VisitFloat(VDP::Float *float_part) {
  VDP::Float *from_float = reinterpret_cast<VDP::Float*>(from_part.get());
  if(from_float->get_value() != (float)NULL){
    printf("found float %s, changing %f to %f\n", float_part->get_name().c_str(), float_part->get_value(), from_float->get_value());
    float_part->set_value(from_float->get_value());
  }
}
void ResponsePacketVisitor::VisitDouble(VDP::Double *double_part) {
  VDP::Double *from_double = reinterpret_cast<VDP::Double*>(from_part.get());
  if(from_double->get_value() != (double)NULL){
    printf("found double\n");
    double_part->set_value(from_double->get_value());
  }
}
void ResponsePacketVisitor::VisitInt64(VDP::Int64 *int64_part) {
  VDP::Int64 *from_int64 = reinterpret_cast<VDP::Int64*>(from_part.get());
  if(from_int64->get_value() != (int64_t)NULL){
    int64_part->set_value(from_int64->get_value());
  }
}
void ResponsePacketVisitor::VisitInt32(VDP::Int32 *int32_part) {
  VDP::Int32 *from_int32 = reinterpret_cast<VDP::Int32*>(from_part.get());
  if(from_int32->get_value() != (int32_t)NULL){
    int32_part->set_value(from_int32->get_value());
  }
}
void ResponsePacketVisitor::VisitInt16(VDP::Int16 *int16_part) {
  VDP::Int16 *from_int16 = reinterpret_cast<VDP::Int16*>(from_part.get());
  if(from_int16->get_value() != (int16_t)NULL){
    int16_part->set_value(from_int16->get_value());
  }
}
void ResponsePacketVisitor::VisitInt8(VDP::Int8 *int8_part) {
  VDP::Int8 *from_int8 = reinterpret_cast<VDP::Int8*>(from_part.get());
  if(from_int8->get_value() != (int8_t)NULL){
    int8_part->set_value(from_int8->get_value());
  }
}

void ResponsePacketVisitor::VisitUint64(VDP::Uint64 *Uint64_part) {
  VDP::Uint64 *from_Uint64 = reinterpret_cast<VDP::Uint64*>(from_part.get());
  if(from_Uint64->get_value() != (uint64_t)NULL){
    Uint64_part->set_value(from_Uint64->get_value());
  }
}
void ResponsePacketVisitor::VisitUint32(VDP::Uint32 *Uint32_part) {
  VDP::Uint32 *from_Uint32 = reinterpret_cast<VDP::Uint32*>(from_part.get());
  if(from_Uint32->get_value() != (uint32_t)NULL){
    Uint32_part->set_value(from_Uint32->get_value());
  }
}
void ResponsePacketVisitor::VisitUint16(VDP::Uint16 *Uint16_part) {
  VDP::Uint16 *from_Uint16 = reinterpret_cast<VDP::Uint16*>(from_part.get());
  if(from_Uint16->get_value() != (uint16_t)NULL){
    Uint16_part->set_value(from_Uint16->get_value());
  }
}
void ResponsePacketVisitor::VisitUint8(VDP::Uint8 *Uint8_part) { 
  VDP::Uint8 *from_Uint8 = reinterpret_cast<VDP::Uint8*>(from_part.get());
  if(from_Uint8->get_value() != (uint8_t)NULL){
    Uint8_part->set_value(from_Uint8->get_value());
  }
}