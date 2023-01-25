#include "../core/include/utils/auto_chooser.h"

/**
  * Initialize the auto-chooser. This class places a choice menu on the brain screen,
  * so the driver can choose which autonomous to run.
  * @param brain the brain on which to draw the selection boxes
  */
AutoChooser::AutoChooser(vex::brain &brain) : brain(brain)
{
  brain.Screen.pressed([](void *ptr){
    AutoChooser &a = *(AutoChooser*)ptr;
    int x = a.brain.Screen.xPosition();
    int y = a.brain.Screen.yPosition();

    entry_t *selected = NULL;
    
    // Check if the touchscreen press is inside each rectangle
    for(int i = 0; i < a.list.size(); i++)
    {
      int rect_x = a.list[i].x, rect_y = a.list[i].y;
      int rect_w = a.list[i].width, rect_h = a.list[i].height;
      if (x > rect_x && x < rect_x + rect_w)
        if (y > rect_y && y < rect_y + rect_h)
          selected = &a.list[i];
    }

    // Re-render all the choices on each press.
    a.render(selected);
    
    if(selected == NULL)
      a.choice = "";
    else
      a.choice = selected->name;

  }, this);
}

#define PADDING 10
#define WIDTH 150
#define HEIGHT 40

/**
  * Place all the autonomous choices on the screen.
  * If one is selected, change it's color
  * @param selected the choice that is currently selected
  */
void AutoChooser::render(entry_t *selected)
{
  for(int i = 0; i < list.size(); i++)
  {
    brain.Screen.setPenColor(vex::color::black);
  
    if(selected != NULL && selected == &list[i])
      brain.Screen.setFillColor(vex::color::white);
    else
      brain.Screen.setFillColor(vex::color::orange);

    brain.Screen.drawRectangle(list[i].x, list[i].y, list[i].width, list[i].height);
    brain.Screen.printAt(list[i].x + PADDING, list[i].y + list[i].height - PADDING, list[i].name.c_str());
  }
}

/**
  * Add a new autonomous option. There are 3 options per row.
  */
void AutoChooser::add(std::string name)
{
  int x = (PADDING * ((list.size() % 3) + 1)) + ((list.size() % 3) * WIDTH);
  int y = (PADDING * ((list.size() / 3) + 1)) + ((list.size() / 3) * HEIGHT);
  entry_t entry = {
    .x=x,
    .y=y,
    .width=WIDTH,
    .height=HEIGHT,
    .name=name
  };

  list.push_back(entry);
  render(NULL);
}

/**
  * Return the selected autonomous
  */
std::string AutoChooser::get_choice()
{
  return choice;
}