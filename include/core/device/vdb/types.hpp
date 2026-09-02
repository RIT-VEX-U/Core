#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include "vex.h"

///Type Ids
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

///Record proto
template <typename... Fields>
class Record;

///Field proto
template <typename T>
class Field;

///TypeIdMap proto 
template <typename T>
struct TypeIdMap;

/// Maps each type -> TypeId

template<typename... Fields>
struct TypeIdMap<Record<Fields...>> {
  static constexpr TypeId value = TypeId::Record;
};

template<>
struct TypeIdMap<bool> {
  static constexpr TypeId value = TypeId::Boolean;
};

template<>
struct TypeIdMap<std::string> {
  static constexpr TypeId value = TypeId::String;
};

template<>
struct TypeIdMap<double> {
  static constexpr TypeId value = TypeId::Double;
};

template<>
struct TypeIdMap<float> {
  static constexpr TypeId value = TypeId::Float;
};

template<>
struct TypeIdMap<uint8_t> {
  static constexpr TypeId value = TypeId::Uint8;
};

template<>
struct TypeIdMap<uint16_t> {
  static constexpr TypeId value = TypeId::Uint16;
};

template<>
struct TypeIdMap<uint32_t> {
  static constexpr TypeId value = TypeId::Uint32;
};

template<>
struct TypeIdMap<uint64_t> {
  static constexpr TypeId value = TypeId::Uint64;
};

template<>
struct TypeIdMap<int8_t> {
  static constexpr TypeId value = TypeId::Int8;
};

template<>
struct TypeIdMap<int16_t> {
  static constexpr TypeId value = TypeId::Int16;
};

template<>
struct TypeIdMap<int32_t> {
  static constexpr TypeId value = TypeId::Int32;
};

template<>
struct TypeIdMap<int64_t> {
  static constexpr TypeId value = TypeId::Int64;
};

/// fiexd point template 
template <TypeId FixedPointType, typename Storage>
struct FixedPoint {
  static constexpr TypeId type = FixedPointType;

  Storage raw_value;
};


/// fixed point aliases

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

template<TypeId Id, typename Storage>
struct TypeIdMap<FixedPoint<Id, Storage>> {
  static constexpr TypeId value = Id;
};

/// primary template to check if a type has an associated id
template<typename T, typename = void>
struct HasTypeId : std::false_type {};

/// actual check for if a type has an associated id
template <typename T>
struct HasTypeId<T, std::void_t<decltype(TypeIdMap<T>::value)>> : std::true_type {};

/**
 * defines a Field
 * A named value that can be serialized and sent to the debug board.
 */
template <typename T>
class Field {
  static_assert(
      HasTypeId<T>::value,
      "Field<T>: T does not have an associated TypeId");
 public:
  /**
   * Creates a Field
   * @param name name for the Field
   * @param value value for the Field to hold; its C++ type determines the VDP Type
   */
  Field(std::string name, T value) : name_(std::move(name)), value_(std::move(value)) {};

  const std::string& get_name() const;

  T& get_value() {
    return value_;
  }

  TypeId get_type() const {
    return TypeIdMap<T>::value;
  }

  template<typename U>
  bool schemas_match(Field<U>& other) const {
    if (name_ != other.get_name() || get_type() != other.get_type()) {
      return false;
    }
    if (get_type() == TypeId::Record) {
      return value_.schemas_match(other.get_value());
    }
    return true;
  }

  template<typename U>
  bool apply_update(Field<U>& other) {
    if (schemas_match(other)) {
      mut.lock();
      value_ = other.get_value();
      mut.unlock();
      return true;
    }
    return false;
  }

 private:
  std::string name_;
  vex::mutex mut;
  T value_;
};

template <typename... Fields>
class Record {
 public:
  explicit Record(Fields... fields) : fields_(std::move(fields)...) {}

  template <std::size_t I>
  const auto& get() const {
    return std::get<I>(fields_);
  }

  static constexpr std::size_t size() {
    return sizeof...(Fields);
  }

  template <typename Function>
  void for_each(Function&& function) {
    std::apply(
        [&](auto&... fields) {
          (function(fields), ...);
        },
        fields_);
  }

  template <typename... OtherFields>
  bool schemas_match(const Record<OtherFields...>& other) const {
    if constexpr (size() != other.size()) {
      return false;
    }
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
      return (get<I>().schemas_match(other.template get<I>()) && ...);
    }(std::index_sequence_for<Fields...>{});
  }
 private:
std::tuple<Fields...> fields_;
};

template <typename... Fields>
Record(Fields...) -> Record<Fields...>;
