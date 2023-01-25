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
/**
  * Initialize the auto-chooser. This class places a choice menu on the brain screen,
  * so the driver can choose which autonomous to run.
  * @param brain the brain on which to draw the selection boxes
  */
  AutoChooser(vex::brain &brain);
  
  /**
   * Add an auto path to the chooser
   * @param name The name of the path. This should be used as an human readable identifier to the auto path
  */
  void add(std::string name);

  /**
   * Get the currently selected auto choice
   * @return the identifier to the auto path
  */
  std::string get_choice();

  protected:

  /**
   *  entry_t is a datatype used to store information that the chooser knows about an auto selection button
   */ 
  struct entry_t
  {
    int x; /**< screen x position of the block*/ 
    int y; /**< screen y position of the block*/ 
    int width; /**< width of the block*/ 
    int height; /**< height of the block*/ 
    std::string name; /**< name of the auto repretsented by the block*/ 
  };

  void render(entry_t *selected);

  std::string choice; /**< the current choice of auto*/
  std::vector<entry_t> list /**< a list of all possible auto choices*/;
  vex::brain &brain; /**< the brain to show the choices on*/


};