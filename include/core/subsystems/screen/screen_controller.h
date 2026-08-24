#include <functional>

namespace ScreenController {

void start(std::function<void()> update, std::function<void()> draw);
void restart();
void stop();

void pause();
void resume();

bool isOn();
bool isPaused();

}