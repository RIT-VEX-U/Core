#pragma once
#include <cstddef>
#include <cstdint>

#define FLASH_READ_DWORD(x) (*(const uint32_t*)(x))

static constexpr uint32_t crc32_table[] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
/**
 * \brief A class for calculating the CRC32 checksum from arbitrary data.
 * \sa http://forum.arduino.cc/index.php?topic=91179.0
 */
class CRC32 {
 public:
  /// @brief Initialize an empty CRC32 checksum.
  constexpr CRC32() { reset(); };

  /// @brief Reset the checksum claculation.
  void constexpr reset() { _state = ~0L; };

  /**
   * @brief Update the current checksum caclulation with the given data.
   * @param data The data to add to the checksum.
   */
  void constexpr update(const uint8_t& data) {
    // via http://forum.arduino.cc/index.php?topic=91179.0
    uint8_t tbl_idx = 0;

    tbl_idx = _state ^ (data >> (0 * 4));
    _state = FLASH_READ_DWORD(crc32_table + (tbl_idx & 0x0f)) ^ (_state >> 4);
    tbl_idx = _state ^ (data >> (1 * 4));
    _state = FLASH_READ_DWORD(crc32_table + (tbl_idx & 0x0f)) ^ (_state >> 4);
  }

  /**
   * @brief Update the current checksum caclulation with the given data.
   * @param Type The data type to read.
   * @param data The data to add to the checksum.
   */
  template <typename Type>
  void update(const Type& data) {
    update(&data, 1);
  }

  /**
   * @brief Update the current checksum caclulation with the given data.
   * @param Type The data type to read.
   * @param data The array to add to the checksum.
   * @param size Size of the array to add.
   */
  template <typename Type>
  void constexpr update(const Type* data, std::size_t size) {
    std::size_t nBytes = size * sizeof(Type);
    const uint8_t* pData = (const uint8_t*)data;

    for (std::size_t i = 0; i < nBytes; i++) {
      update(pData[i]);
    }
  }

  /// @return the caclulated checksum.
  uint32_t constexpr finalize() const { return ~_state; }

  /**
   * @brief Calculate the checksum of an arbitrary data array.
   * @param Type The data type to read.
   * @param data A pointer to the data to add to the checksum.
   * @param size The size of the data to add to the checksum.
   * @return the calculated checksum.
   */
  template <typename Type>
  static constexpr uint32_t calculate(const Type* data, std::size_t size) {
    CRC32 crc;
    crc.update(data, size);
    return crc.finalize();
  }

 private:
  /// @brief The internal checksum state.
  uint32_t _state = ~0L;
};
