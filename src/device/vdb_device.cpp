#include "core/device/vdb_device.hpp"

#include <cstddef>
#include <cstdint>
namespace VDB {
/**
 * delay for ms time
 * @param ms the ms to delay for
 */
void delay_ms(uint32_t ms) { vexDelay(ms); }
/**
 * @return the time in ms of the bot since startup
 */
uint32_t time_ms() { return vexSystemTimeGet(); }
}  // namespace VDB
