#pragma once
#include "vex.h"
#include <string>
#include <vector>


/**
 * Autochooser is a utility to make selecting robot autonomous programs easier
 * source: RIT VexU Wiki
 * During a season, we usually code between 4 and 6 autonomous programs. 
 * Most teams will change their entire robot program as a way of choosing autonomi
 * but this may cause issues if you have an emergency patch to upload during a competition. 
 * This class was built as a way of using the robot screen to list autonomous programs, and the touchscreen to select them.
 */
class AutoChooser
{
  public:

  AutoChooser(vex::brain &brain);

  void add(std::string name);
  std::string get_choice();

  protected:

  // entry_t is a datatype used to store information that the chooser knows about an auto selection button
  struct entry_t
  {
    int x, y, width, height;
    std::string name;
  };

  void render(entry_t *selected);
  std::string choice;
  std::vector<entry_t> list;
  vex::brain &brain;


};