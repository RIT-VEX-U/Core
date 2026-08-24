#include "vex.h"
#include "core/subsystems/screen/legacy.h"

// Private functions from the VEX V5 API Build that need to be declared here to use
extern "C" {
uint32_t* vexDisplayOffscreenBufferGet(uint32_t width, uint32_t height, uint32_t pixelSize);
void vexDisplayOffscreenBufferDestroy(uint32_t* buffer);
void vexDisplayOffscreenBufferPixelSet(uint32_t* buffer, uint32_t x, uint32_t y, uint32_t color);
uint32_t vexDisplayOffscreenBufferPixelGet(uint32_t* buffer, uint32_t x, uint32_t y);
void vexDisplayOffscreenBufferRectDraw(uint32_t* buffer, uint32_t x, uint32_t y, uint32_t width, uint32_t height);
void vexDisplayOffscreenBufferRectFill(uint32_t* buffer, uint32_t x, uint32_t y, uint32_t width, uint32_t height);
void vexDisplayOffscreenBufferScrollH(uint32_t* buffer, uint32_t pixels);
void vexDisplayOffscreenBufferBlit(uint32_t* buffer, uint32_t x, uint32_t y, uint32_t width, uint32_t height);
}

namespace ScreenController {

} // namespace ScreenController