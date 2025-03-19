#include "core/subsystems/screen.h"
#include "pl_mpeg.h"
#include <string>

/// @brief Only one video file can be open at a time due to memory constraints
void set_video(const std::string &filename);
/// @brief  restart the global video
void video_restart();
// plays the video set by set_video()
// because of memory constraints we're limited to one video at a time
class VideoPlayer : public screen::Page {
  public:
    VideoPlayer();
    void update(bool was_pressed, int x, int y) override;

    void draw(vex::brain::lcd &screen, bool first_draw, unsigned int frame_number) override;
};