#pragma once
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include <span>

#include "vex.h"

namespace VDP {
using Packet = std::vector<uint8_t>;

/// Type Ids
enum class TypeId : uint8_t {
  Record = 0,
  Boolean = 1,
  String = 2,

  Double = 3,
  Float = 4,

  Uint8 = 5,
  Uint16 = 6,
  Uint32 = 7,
  Uint64 = 8,

  Int8 = 9,
  Int16 = 10,
  Int32 = 11,
  Int64 = 12,

  Q3_4 = 13,
  Q4_4 = 14,
  Q7_1 = 15,
  Q1_7 = 16,
  Q6_2 = 17,
  Q2_6 = 18,
  Q7_8 = 19,
  Q8_8 = 20,
  Q15_1 = 21,
  Q1_15 = 22,
  Q10_6 = 23,
  Q9_6 = 24,
  Q12_12 = 25,
  Q16_8 = 26,
  Q8_16 = 27,
  Q15_16 = 28,
  Q16_16 = 29,
  Q24_8 = 30,
  Q8_24 = 31,
  Q31_32 = 32,
  Q32_32 = 33,

  UNKNOWN = 34,
};

/**
 * @return a Type Ids value as a string
 */
std::string to_string(TypeId t);

/// Recprd proto
template <typename... Fields>
class Record;

/// Field proto
template <typename T>
class Field;

// TypeIdMap proto
template <typename T>
struct TypeIdMap;

/// Maps each type -> TypeId

template <typename... Fields>
struct TypeIdMap<Record<Fields...>> {
  static constexpr TypeId value = TypeId::Record;
};

template <>
struct TypeIdMap<bool> {
  static constexpr TypeId value = TypeId::Boolean;
};

template <>
struct TypeIdMap<std::string> {
  static constexpr TypeId value = TypeId::String;
};

template <>
struct TypeIdMap<double> {
  static constexpr TypeId value = TypeId::Double;
};

template <>
struct TypeIdMap<float> {
  static constexpr TypeId value = TypeId::Float;
};

template <>
struct TypeIdMap<uint8_t> {
  static constexpr TypeId value = TypeId::Uint8;
};

template <>
struct TypeIdMap<uint16_t> {
  static constexpr TypeId value = TypeId::Uint16;
};

template <>
struct TypeIdMap<uint32_t> {
  static constexpr TypeId value = TypeId::Uint32;
};

template <>
struct TypeIdMap<uint64_t> {
  static constexpr TypeId value = TypeId::Uint64;
};

template <>
struct TypeIdMap<int8_t> {
  static constexpr TypeId value = TypeId::Int8;
};

template <>
struct TypeIdMap<int16_t> {
  static constexpr TypeId value = TypeId::Int16;
};

template <>
struct TypeIdMap<int32_t> {
  static constexpr TypeId value = TypeId::Int32;
};

template <>
struct TypeIdMap<int64_t> {
  static constexpr TypeId value = TypeId::Int64;
};

/**
 * Since our build system makes int32_t an alias for long, we convert regular ints to in32_ts
 */
template <typename T>
  requires std::is_same_v<T, int>
struct TypeIdMap<T> {
  static_assert(sizeof(T) == sizeof(int32_t), "VDP requires a 32-bit int");
  static constexpr TypeId value = TypeId::Int32;
};

/// Template for fixed point types
template <TypeId FixedPointType, typename Storage>
struct FixedPoint {
  static constexpr TypeId type = FixedPointType;

  Storage raw_value;
};

/// Aliases for fixed point types

using Q3_4 = FixedPoint<TypeId::Q3_4, int8_t>;
using Q4_4 = FixedPoint<TypeId::Q4_4, uint8_t>;
using Q7_1 = FixedPoint<TypeId::Q7_1, int8_t>;
using Q1_7 = FixedPoint<TypeId::Q1_7, int8_t>;
using Q6_2 = FixedPoint<TypeId::Q6_2, int8_t>;
using Q2_6 = FixedPoint<TypeId::Q2_6, int8_t>;

using Q7_8 = FixedPoint<TypeId::Q7_8, int16_t>;
using Q8_8 = FixedPoint<TypeId::Q8_8, uint16_t>;
using Q15_1 = FixedPoint<TypeId::Q15_1, int16_t>;
using Q1_15 = FixedPoint<TypeId::Q1_15, int16_t>;
using Q10_6 = FixedPoint<TypeId::Q10_6, uint16_t>;
using Q9_6 = FixedPoint<TypeId::Q9_6, int16_t>;

using Q12_12 = FixedPoint<TypeId::Q12_12, int32_t>;
using Q16_8 = FixedPoint<TypeId::Q16_8, int32_t>;
using Q8_16 = FixedPoint<TypeId::Q8_16, int32_t>;
using Q15_16 = FixedPoint<TypeId::Q15_16, int32_t>;
using Q16_16 = FixedPoint<TypeId::Q16_16, uint32_t>;
using Q24_8 = FixedPoint<TypeId::Q24_8, int32_t>;
using Q8_24 = FixedPoint<TypeId::Q8_24, int32_t>;

using Q31_32 = FixedPoint<TypeId::Q31_32, int64_t>;
using Q32_32 = FixedPoint<TypeId::Q32_32, uint64_t>;

/// Maps fixed points to their Type Ids
template <TypeId Id, typename Storage>
struct TypeIdMap<FixedPoint<Id, Storage>> {
  static constexpr TypeId value = Id;
};

/// Primary template to check if a type has an associated id
template <typename T, typename = void>
struct HasTypeId : std::false_type {};

/// Actual check for if a type has an associated id
template <typename T>
struct HasTypeId<T, std::void_t<decltype(TypeIdMap<T>::value)>> : std::true_type {};

/// Check for if a held field type is a Field
template <typename T>
struct IsField : std::false_type {};

template <typename T>
struct IsField<Field<T>> : std::true_type {};

template <typename... Fields>
struct IsField<Record<Fields...>> : std::true_type {};
template <typename... Fields>
  requires(IsField<std::remove_cvref_t<Fields>>::value && ...)
struct TypeIdMap<std::tuple<Fields...>> {
  static constexpr TypeId value = TypeId::Record;
};

/// Check for if a held field type is a fixed point type
template <typename T>
struct IsFixedPoint : std::false_type {};

template <TypeId Id, typename Storage>
struct IsFixedPoint<FixedPoint<Id, Storage>> : std::true_type {};

/*
 * defines a Field
 * A named value that can be serialized and sent to the debug board.
 */
template <typename T>
class Field {
  static_assert(HasTypeId<T>::value, "Field<T>: T does not have an associated TypeId");

 public:
  /**
   * @brief Constructor for a field, a form of data to be sent to the debug board
   * @param name name for the Field
   * @param value value for the Field to hold; its C++ type determines the VDP Type
   */
  // The referenced value must outlive this field.
  Field(std::string name, T& value) : name_(std::move(name)), value_(value) {};

  /**
   * @breif Gets the name of the field
   * @return the field name
   */
  const std::string& get_name() const { return name_; };

  /**
   * @brief Gets the value currently stored by the field
   * @return the value currently stored by the field
   */
  const T& get_value() const { return value_; }

  /**
   * @brief Gets the type of the field in the form of a TypeId enum
   * @return the type id of the field
   */
  const TypeId get_type() const { return TypeIdMap<T>::value; }

  /**
   * @brief serializes the field's schema in the form of a VDP::Packet
   * @return the serialized schema
   */
  VDP::Packet serialize_schema() const {
    VDP::Packet out;
    out.push_back(static_cast<uint8_t>(get_type()));
    out.insert(out.end(), name_.begin(), name_.end());
    out.push_back(0);

    return (out);
  }

  /**
   * @brief serializes the field's data in the form of a VDP::Packet
   * @return the serialized data
   */
  VDP::Packet serialize_data() const {
    /// Since strings are not fixed length, we need to use a 0 byte to signal when it has ended
    if constexpr(std::is_same_v<T, std::string>) {
      VDP::Packet out(value_.begin(), value_.end());
      out.push_back(0);
      return out;
    }
    else {
      VDP::Packet out(sizeof(T));
      std::memcpy(out.data(), &value_, sizeof(T));
      return out;
    }
  }

  /**
   * @brief deserializes a packet's data and applies it to the field
   * @return the number of bytes read in the packet
   */
  size_t apply_update(const VDP::Packet& packet_in) {
    if constexpr (std::is_same_v<T, std::string>) {
      /// if the field holds a string, find the 0 delimiter and and get the string for that length of the packet
      auto str_end = std::find(packet_in.begin(), packet_in.end(), uint64_t(0));

      auto length = str_end - packet_in.begin();

      mut.lock();
      value_.assign(packet_in.begin(), str_end);
      mut.unlock();

      return length + 1;
    }
    else {
      /// if the field is a value that is not of variable size, just copy the data into the field's data
      mut.lock();
      std::memcpy(&value_, packet_in.data(), sizeof(T));
      mut.unlock();

      return sizeof(T);
    }
  }

  /**
   * @brief formats the field's data as a string
   * @return a string representation of the field's data
   */
  std::string data_to_string(std::size_t depth = 0) const {
    // add the name and a specified number of indents to the string
    std::string out = std::string(depth * 2, ' ') + name_ + " : ";

    // check the held type
    if constexpr (std::is_same_v<T, std::string>) {
      // if it is a string just add the value directly
      return out + value_;
    } else if constexpr (std::is_same_v<T, bool>) {
      // if it is a boolean translate it to a true or false string
      return out + (value_ ? "true" : "false");
    } else if constexpr (std::is_integral_v<T>) {
      /*
       * if it is an integral check if it is signed or unsigned,
       * convert to uint64_t or int64_t accordingly, and get the string
       * format of that
       */
      if (std::is_signed_v<T>) {
        return out + std::to_string(static_cast<int64_t>(value_));
      } else {
        return std::to_string(static_cast<uint64_t>(value_));
      }
    } else if constexpr (std::is_floating_point_v<T>) {
      /// if it is a floating point we can just cast it to a string
      return std::to_string(value_);
    } else if constexpr (IsFixedPoint<T>::value) {
      /* if it is a fixed point check if it is signed or unsigned,
       * cast the raw byte value of the data to an int64_t or uint64_t,
       * and then get the string format of that
       */
      if constexpr (std::is_signed_v<decltype(value_.raw_value)>) {
        return std::to_string(static_cast<int64_t>(value_.raw_value));
      } else {
        return std::to_string(static_cast<uint64_t>(value_.raw_value));
      }
    } else {
      // if none of those work then we do not support this type
      static_assert(std::is_same_v<T, void>, "data_to_string does not support this type");
    }
  }

  /**
   * formats the field's schema as a string
   * @return the string representation of the field's schema
   */
  std::string schema_to_string(std::size_t depth = 0) const {
    return std::string(depth * 2, ' ') + name_ + " : " + VDP::to_string(get_type());
  }


 protected:
  std::string name_;
  vex::mutex mut;
  T& value_;
};

/// Preserve the original variable's type when binding a reference.
template <typename T>
Field(std::string, T&) -> Field<T>;

/**
 * Defines a record
 * A record owns a tuple of references to existing fields or records.
 */
template <typename... Fields>
class Record {
  /// checks that each element being input is a Field or a Record
  static_assert((IsField<std::remove_cvref_t<Fields>>::value && ...),
                "Record elements must all be Field or Record objects");

 public:
  /**
   * @brief Constructs a Record, a field that contains a tuple of other fields
   * @param name the name for the Record
   * @param Fields a list of fields for the record to hold
   */
  // The referenced fields must outlive this record.
  explicit Record(std::string name, Fields&... fields)
      : name_(std::move(name)), value_(fields...) {}

  const std::string& get_name() const { return name_; }
  TypeId get_type() const { return TypeId::Record; }
  const auto& get_value() const { return value_; }

  /**
   * @return the size of the held tuple
   */
  static constexpr std::size_t size() { return sizeof...(Fields); }

  /**
   * @return the field at a specified index of the held tuple
   */
  template <std::size_t I>
  const auto& get() const {
    return std::get<I>(value_);
  }

  /**
   * @brief serializes the record's schema in the form of a VDP::Packet
   * @return the serialized schema
   */
  VDP::Packet serialize_schema() const {
    static_assert(sizeof...(Fields) <= 255, "A record cannot contain more than 255 fields");
    Packet out;

    // add the type byte
    out.push_back(static_cast<uint8_t>(TypeId::Record));

    // add the name
    out.insert(out.end(), this->get_name().begin(), this->get_name().end());
    out.push_back(0);

    out.push_back((static_cast<uint8_t>(size())));

    // use std::apply to loop through the fields in the tuple
    std::apply(
        [&](const auto&... fields) {
          (
              [&] {
                // add the field's serialized schema to the packet
                Packet field_schema = fields.serialize_schema();

                out.insert(out.end(), field_schema.begin(), field_schema.end());
              }(),
              ...);
        },
        get_value());
    return out;
  }

  /**
   * @brief serializes the record's data in the form of a VDP::Packet
   * @return the serialized data
   */
  VDP::Packet serialize_data() const {
    VDP::Packet out;

    std::apply(
        [&](const auto&... fields) {
          (
              [&] {
                // add the field's serialized data to the packet
                Packet field_schema = fields.serialize_data();

                out.insert(out.end(), field_schema.begin(), field_schema.end());
              }(),
              ...);
        },
        get_value());
    return out;
  }

  /**
   * @breif deserializes a packet of data and applies it to the Record's fields
   */
  size_t apply_update(VDP::Packet in) {
    std::size_t offset = 0;

    std::apply(
      [&](auto&... fields) {
        ([&] {
          auto read_bytes = fields.apply_update(VDP::Packet(in.begin() + offset, in.end()));
          offset += read_bytes;
        }(),
        ...);
      },
      get_value());
    return offset;
  }

  /**
   * @brief converts a record's held data to a human readable string
   * @return a string representation of the record's held data
   */
  std::string data_to_string(std::size_t depth = 0) const {
    std::string out = std::string(depth * 2, ' ') + this->get_name() + " : {\n";

    /// loop through each field held within the record and add their data strings, increasing the depth
    std::apply([&](const auto&... fields) { ([&] { out += fields.data_to_string(depth + 1) += ",\n"; }(), ...); },
               this->value_);
    out += std::string(depth * 2, ' ') + "}";
    return out;
  }

  /**
   * @brief converts a record's schema to a human readable string
   * @return a string representation of the record's schema
   */
  std::string schema_to_string(std::size_t depth = 0) const {
    std::string out = std::string(depth * 2, ' ') + this->get_name() + " : record {\n";

    /// loop through each field held within the record and add their schema strings, increasing the depth
    std::apply([&](const auto&... fields) { ([&] { out += fields.schema_to_string(depth + 1) + ",\n"; }(), ...); },
               this->value_);
    out += std::string(depth * 2, ' ') + "}";
    return out;
  }

 private:
  std::string name_;
  std::tuple<Fields&...> value_;
};

/// deduction guide for Records so that you don't need to use template arguments when creating one
template <typename... Fields>
Record(std::string, Fields&...) -> Record<Fields...>;

}  // namespace VDP
