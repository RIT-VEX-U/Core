#pragma once
#include <cstdint>
#include <cstddef>

/// \brief A class for calculating the CRC32 checksum from arbitrary data.
/// \sa http://forum.arduino.cc/index.php?topic=91179.0
class CRC32 {
  public:
    /**
    * @brief Initialize an empty CRC32 checksum.
    */
    CRC32();
    /**
     * @brief Reset the checksum claculation.
     */
    void reset();
    /**
     * @brief Update the current checksum caclulation with the given data.
     * @param data The data to add to the checksum.
     */
    void update(const uint8_t &data);
    /** 
     * @brief Update the current checksum caclulation with the given data.
     * @param Type The data type to read.
     * @param data The data to add to the checksum.
    */
    template <typename Type> void update(const Type &data) { update(&data, 1); }
    /**
     * @brief Update the current checksum caclulation with the given data.
     * @param Type The data type to read.
     * @param data The array to add to the checksum.
     * @param size Size of the array to add.
     */
    template <typename Type> void update(const Type *data, std::size_t size) {
        std::size_t nBytes = size * sizeof(Type);
        const uint8_t *pData = (const uint8_t *)data;

        for (std::size_t i = 0; i < nBytes; i++) {
            update(pData[i]);
        }
    }
    /**
     * @return the caclulated checksum.
     */
    uint32_t finalize() const;
    /**
     * @brief Calculate the checksum of an arbitrary data array.
     * @param Type The data type to read.
     * @param data A pointer to the data to add to the checksum.
     * @param size The size of the data to add to the checksum.
     * @return the calculated checksum.
     */
    template <typename Type> static uint32_t calculate(const Type *data, std::size_t size) {
        CRC32 crc;
        crc.update(data, size);
        return crc.finalize();
    }

  private:
  /**
   * @brief The internal checksum state.
   */
    uint32_t _state = ~0L;
};
