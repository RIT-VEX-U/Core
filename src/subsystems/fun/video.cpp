#include "core/subsystems/fun/video.h"

#include <cstdint>

#define PL_MPEG_IMPLEMENTATION
#include "core/subsystems/fun/pl_mpeg.h"

static uint8_t buf[358400];

enum VideoState { DoesntExist, TooBig, DidntReadRight, Ok, NeverInitialized };

static vex::task video_task;
static VideoState state = VideoState::NeverInitialized;
static std::string name = "";
static uint8_t* argb_buffer;
static int w;
static int h;
static int x;
static int y;
static bool frame_ready = false;
static plm_t* plm;
const int32_t video_player_priority = vex::thread::threadPrioritylow;
static int32_t file_size = 0;
static bool should_restart = false;

void video_restart() { should_restart = true; }

void explain_error(vex::brain::lcd& screen) {
  switch (state) {
    case DoesntExist:
      screen.printAt(40, 30, true, "Couldn't find video %s", name.c_str());
      break;
    case TooBig:
      screen.printAt(40, 30, true, "%s was too big to open", name.c_str());
      break;
    case DidntReadRight:
      screen.printAt(40, 30, true, "%s wasnt read correctly. Are you sure its mpeg1", name.c_str());
      break;
    case NeverInitialized:
      screen.printAt(40, 30, true, "no video loaded. did you forget set_video()");
      break;

    default:
      break;
  }
}

int video_player() {
  if (state != Ok) {
    return 1;
  }
  plm_frame_t* frame = NULL;
  while (true) {
    for (int i = 1; (frame = plm_decode_video(plm)); i++) {
      uint32_t start_ms = vex::timer::system();

      frame_ready = false;
      plm_frame_to_bgra(frame, argb_buffer, w * 4);
      frame_ready = true;
      uint32_t elapsed_ms = vex::timer::system() - start_ms;

      vexDelay(33 - elapsed_ms);
      if (should_restart) {
        should_restart = false;
        break;
      }
    }
    plm_rewind(plm);
  }
  return 0;
}

void set_video(const std::string& filename) {
  vex::brain Brain;
  const char* fname = filename.c_str();

  if (!Brain.SDcard.exists(fname)) {
    state = DoesntExist;
    return;
  }
  file_size = Brain.SDcard.size(fname);
  if (file_size > sizeof(buf)) {
    state = TooBig;
    return;
  }
  const int32_t read = Brain.SDcard.loadfile(fname, buf, sizeof(buf));

  if (read != file_size) {
    state = DidntReadRight;
    return;
  }
  plm = plm_create_with_memory(buf, file_size, 0);

  plm_set_audio_enabled(plm, false);
  plm_set_loop(plm, true);

  w = plm_get_width(plm);
  h = plm_get_height(plm);
  int render_buf_len = w * h * 4;
  x = (480 - w) / 2;
  y = (200 - h) / 2;
  argb_buffer = new uint8_t[render_buf_len];
  state = Ok;
  name = fname;
  video_task = vex::task(video_player, video_player_priority);
}

VideoPlayer::VideoPlayer() {}
void VideoPlayer::update(bool was_pressed, int x, int y) {}

void VideoPlayer::draw(vex::brain::lcd& screen, bool first_draw, unsigned int frame_number) {
  if (state != Ok) {
    explain_error(screen);
    return;
  }
  // nothin bad ever happened with a spin lock
  for (int i = 0; i < 5; i++) {
    if (frame_ready) {
      break;
    }
    vexDelay(1);
  }
  screen.drawImageFromBuffer((uint32_t*)argb_buffer, x, y, w, h);
}