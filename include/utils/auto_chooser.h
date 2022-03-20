#pragma once
#include "vex.h"
#include <string>
#include <vector>

class AutoChooser
{
  public:
  AutoChooser(vex::brain &brain);

  void add(std::string name);
  std::string get_choice();

  protected:

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