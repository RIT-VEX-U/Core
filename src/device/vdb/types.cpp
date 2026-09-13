#include "core/device/vdb/types.hpp"

namespace VDP {
std::string to_string(TypeId t) {
  switch (t) {
    case TypeId::Record:
      return "record";
    case TypeId::Boolean:
      return "boolean";
    case TypeId::String:
      return "string";

    case TypeId::Float:
      return "float";
    case TypeId::Double:
      return "double";

    case TypeId::Uint8:
      return "uint8";
    case TypeId::Uint16:
      return "uint16";
    case TypeId::Uint32:
      return "uint32";
    case TypeId::Uint64:
      return "uint64";

    case TypeId::Int8:
      return "int8";
    case TypeId::Int16:
      return "int16";
    case TypeId::Int32:
      return "int32";
    case TypeId::Int64:
      return "int64";

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
