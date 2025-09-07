#pragma once
#include <string>
#include <vector>

#include "core/subsystems/screen.h"
#include "core/utils/geometry.h"
#include "vex.h"

/**
 * Autochooser is a utility to make selecting robot autonomous programs easier
 * source: RIT VexU Wiki
 * During a season, we usually code between 4 and 6 autonomous programs.
 * Most teams will change their entire robot program as a way of choosing autonomi
 * but this may cause issues if you have an emergency patch to upload during a competition.
 * This class was built as a way of using the robot screen to list autonomous programs, and the touchscreen to select
 * them.
 */
class AutoChooser : public screen::Page {
 public:
  /**
   * Initialize the auto-chooser. This class places a choice menu on the brain screen,
   * so the driver can choose which autonomous to run.
   * @param brain the brain on which to draw the selection boxes
   */
  AutoChooser(std::vector<std::string> paths, size_t def = 0);

  void update(bool was_pressed, int x, int y);
  void draw(vex::brain::lcd&, bool first_draw, unsigned int frame_number);

  /**
   * Get the currently selected auto choice
   * @return the identifier to the auto path
   */
  size_t get_choice();

 protected:
  /**
   *  entry_t is a datatype used to store information that the chooser knows about an auto selection button
   */
  struct entry_t {
    Rect rect;
    std::string name; /**< name of the auto repretsented by the block*/
  };

  static const size_t width = 380;
  static const size_t height = 220;

  size_t choice; /**< the current choice of auto*/
  std::vector<entry_t> list /**< a list of all possible auto choices*/;
};