#include "../core/include/utils/formatting.h"

std::string double_to_string(double value, int decimal_places) {
    char buf[100]; // should be more than enough
    snprintf(buf, 100, "%.*f", decimal_places, value);
    return std::string{buf};
}