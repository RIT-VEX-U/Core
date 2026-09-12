#include "core/utils/math/geometry/translation2d.h"
#include "vex.h"
#include <v5_apitypes.h>
#include "core/subsystems/screen/screen_controller.h"

namespace ScreenController {

namespace {

/// The thread in which the Screen updates
vex::task* screen_task = nullptr;

/// Screen task handle callback and buffer
std::function<screen_handle> handle_callbacks[2] = {nullptr, nullptr};

/// Screen task draw callback and buffer
std::function<screen_handle> draw_callbacks[2] = {nullptr, nullptr};

/// Boolean flags for controlling the ScreenController.
union {
    /// Field for accessing the individual flag bits of the ScreenController
    struct {
        uint8_t buffer_ready : 1;   /// Determines if the buffered data is ready to be loaded
        uint8_t callback_index : 1; /// Denotes which handle buffers are actively being used
        uint8_t turning_off : 1;    /// Denotes if the ScreenController should "turn off", or pause on a blank screen
        uint8_t turned_off : 1;     /// Denotes if the ScreenController is "turned off", or paused on a blank screen
        uint8_t paused_buffer : 1;  /// Buffer denoting if the screen should be paused for the next frame
        uint8_t paused : 1;         /// Denotes if the screen is currently paused
        uint8_t screen_touch : 1;   /// Denotes if the screen is being touched
        // extra
    } bits;

    /// Field for accessing the flags collectively as a whole byte
    uint8_t byte;
} controller_flags = {.byte=0};

/// Unsigned int for storing the number of frames has occurred since the screen started.
uint64_t frame_count;

/// Timestamp of the previous frame used to determine frame delta time
uint64_t last_frame_time;

/// Timestamp of the current frame used to determine frame delta time
uint64_t current_frame_time;

/// Stores the latest pressed location on the brain screen.
Translation2d curr_touch_pos;

/// Stores the previous pressed location on the brain screen.
Translation2d last_touch_pos;

/// Stores the first pressed location of the current action on the brain screen.
Translation2d first_touch_pos;

/// The primary callback of the ScreenController task
int screen_task_func() {
    vexDisplayBackgroundColor(ClrBlack);
    vexTouchUserCallbackSet([](V5_TouchEvent te, int32_t x, int32_t y) {
        if(te == kTouchEventPressAuto && controller_flags.bits.screen_touch) {
            last_touch_pos = curr_touch_pos;
            curr_touch_pos = Translation2d(x, y);
        } else if(te == kTouchEventPress) {
            first_touch_pos = last_touch_pos = curr_touch_pos = Translation2d(x, y);
            controller_flags.bits.screen_touch = 1;
        } else { // kTouchEventRelease
            controller_flags.bits.screen_touch = 0;
        }
    });

    while(true) {
        // Updates times
        last_frame_time = current_frame_time;
        current_frame_time = vexSystemHighResTimeGet();

        // Handle if the brain should turn off
        if(controller_flags.bits.turning_off) {
            handle_callbacks[0] = handle_callbacks[1] = nullptr;
            draw_callbacks[0] = draw_callbacks[1] = nullptr;
            controller_flags.bits.buffer_ready = 0;
            controller_flags.bits.paused_buffer = 0;
            controller_flags.bits.turning_off = 0;
            controller_flags.bits.turned_off = 1;
            frame_count = 0;

            // draw blank screen
            vexDisplayErase();
            vexDisplayRender(true, true);
        }

        // Checks to see if the callbacks should change
        if(controller_flags.bits.buffer_ready) {
            controller_flags.bits.callback_index = !controller_flags.bits.callback_index;
            controller_flags.bits.screen_touch = 0;
            controller_flags.bits.buffer_ready = 0;
            frame_count = 0;
            last_frame_time = current_frame_time;
        }

        /* Updates the screen pause status
           If the screen changes, it should run for one frame to ensure that the previous effects of the previous
           callbacks are replaced. As such, the following condition is an else-if, as that prevents the screen from
           being set and paused without a frame passing. */ 
        else if(controller_flags.bits.paused != controller_flags.bits.paused_buffer) {
            last_frame_time = current_frame_time;
            controller_flags.bits.paused = controller_flags.bits.paused_buffer;
        }

        // Skips to next frame if the screen should be paused or off
        if(controller_flags.bits.paused || controller_flags.bits.turned_off) {
            vexDelay(50);
            continue;
        }   

        // Update
        if(handle_callbacks[controller_flags.bits.callback_index])
            handle_callbacks[controller_flags.bits.callback_index]();

        // Draw
        if(draw_callbacks[controller_flags.bits.callback_index])
            draw_callbacks[controller_flags.bits.callback_index]();
        vexDisplayRender(true, true);
        
        // Finishing frame
        frame_count++;
        vexDelay(5);
    }

    return 0;
}

} // namespace

bool set(std::function<screen_handle> handle, std::function<screen_handle> draw) {
    handle_callbacks[!controller_flags.bits.callback_index] = handle;
    draw_callbacks[!controller_flags.bits.callback_index] = draw;
    controller_flags.bits.paused_buffer = 0;
    controller_flags.bits.turning_off = 0;
    controller_flags.bits.buffer_ready = 1;
    
    if(screen_task == nullptr) {
        screen_task = new vex::task(screen_task_func);
    }

    return screen_task == nullptr;
}

void unset() {
    controller_flags.bits.turning_off = 1;
}

bool was_initialized() {
    return screen_task != nullptr;
}

void pause() {
    controller_flags.bits.paused_buffer = 1;
}

void resume() {
    controller_flags.bits.paused_buffer = 0;
}

bool is_running() {
    return was_initialized() && !controller_flags.bits.paused && !controller_flags.bits.turned_off;
}

std::optional<Translation2d> get_press_pos() {
    if(is_running() && controller_flags.bits.screen_touch) return curr_touch_pos;
    return std::nullopt;
}

std::optional<Translation2d> get_last_press_pos() {
    if(is_running() && controller_flags.bits.screen_touch) return last_touch_pos;
    return std::nullopt;
}

std::optional<Translation2d> get_first_press_pos() {
    if(is_running() && controller_flags.bits.screen_touch) return first_touch_pos;
    return std::nullopt;
}

uint64_t get_frame_count() {
    return frame_count;
}

uint64_t get_frame_delta_time() {
    return is_running() * (current_frame_time - last_frame_time);
}

} // namespace ScreenController