#pragma once

#include <functional>
#include <optional>

#include "vex.h"
#include "core/utils/math/geometry/translation2d.h"

namespace ScreenController {

/// Type used for handle (or update) and draw callbacks by ScreenController functions.
using screen_handle = void();

/**
 * Ensures the ScreenController's thread is running, and sets the ScreenController's callbacks and data. Use this to
 * both start and reset the ScreenController.
 * @param handle A callback function for handling/updating the screen. This runs before the draw callback.
 * @param draw An optional callback function for drawing to the screen. This runs after the handle callback.
 * @returns FALSE if the ScreenController can be started without error, and TRUE if it cannot (usually because it has
 * already been started)
 */
bool set(std::function<screen_handle> handle, std::function<screen_handle> draw = nullptr);

/// Unsets the ScreenController's callbacks and data. Use this to stop the screen, effectively turning it off.
void unset();

/**
 * Returns whether the screen has been initialized with the ScreenController functions or not
 * @returns TRUE if the screen has been initialized, and FALSE if it has not been
 */
bool was_initialized();

/// Pauses the ScreenController thread after the current frame.
void pause();

/// Resumes the ScreenController thread.
void resume();

/**
 * Returns whether the screen is actively updating and drawing with the ScreenController functions or not
 * @returns TRUE if the screen was initialized and is running, and FALSE if it is either paused or not initialized.
 */
bool is_running();

/**
 * If the screen is running and being touched, this will return the currently pressed coordinates.
 * @returns An optional that, when the screen is running and being touched, stores a Translation2d object representing
 * the current coordinate.
 */
std::optional<Translation2d> get_press_pos();

/**
 * If the screen is running and being held, this will return the last coordinates pressed.
 * @returns An optional that, when the screen is running and being held, stores a Translation2d object representing
 * the previous coordinate it was held at.
 */
std::optional<Translation2d> get_last_press_pos();

/**
 * If the screen is running and being touched, this will return the first coordinates pressed as part of the action
 * @returns An optional that, when the screen is running and being touched, stores a Translation2d object representing
 * the first coordinates to be held during the touch.
 */
std::optional<Translation2d> get_first_press_pos();

/**
 * Returns the current frame number since the screen has been started.
 * @returns An unsigned integer representing the amount of frames that have occurred since the screen was started.
 */
uint64_t get_frame_count();

/**
 * Returns the amount of time that has passed since the previous frame of the ScreenController.
 * @returns An unsigned 64-bit integer representing the amount of microseconds that has passed since the previous frame
 */
uint64_t get_frame_delta_time();

} // namespace ScreenController