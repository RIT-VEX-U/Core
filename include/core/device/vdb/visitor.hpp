#pragma once
#include "core/device/vdb/protocol.hpp"
#include "core/device/vdb/types.hpp"

class ResponsePacketVisitor : public VDP::Visitor {
public:
  ResponsePacketVisitor(VDP::PartPtr from_part);

  void VisitRecord(VDP::Record *record) override;

  void VisitString(VDP::String *str) override;

  void VisitFloat(VDP::Float *float_part) override;
  void VisitDouble(VDP::Double *double_part) override;

  void VisitInt64(VDP::Int64 *int64_part) override;
  void VisitInt32(VDP::Int32 *int32_part) override;
  void VisitInt16(VDP::Int16 *int16_part) override;
  void VisitInt8(VDP::Int8 *int8_part) override;

  void VisitUint64(VDP::Uint64 *Uint64_part) override;
  void VisitUint32(VDP::Uint32 *Uint32_part) override;
  void VisitUint16(VDP::Uint16 *Uint16_part) override;
  void VisitUint8(VDP::Uint8 *Uint8_part) override;

private:
VDP::PartPtr from_part;
};