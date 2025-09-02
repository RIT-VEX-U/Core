#include "core/utils/controls/take_back_half.h"
#include "core/utils/math_util.h"

TakeBackHalf::TakeBackHalf(double gain, double first_cross_split, double thresh)
    : TBH_gain(gain), first_cross_split(first_cross_split), on_target_threshhold(fabs(thresh)) {
    tbh = 0.0;
    output = 0.0;
    prev_error = 0.0;
    lower = 0.0;
    upper = 0.0;
    first_cross = true;
}

void TakeBackHalf::init(double start_pt, double set_pt) {
    if (set_pt == target) {
        // nothing to do
        return;
    }
    first_cross = true;
    tbh = output;
    target = set_pt;
    output = update(start_pt);
}

double TakeBackHalf::update(double val) {

    if (target == 0.0) {
        printf("no target\n");
        return 0.0;
    }

    double error = target - val;
    output += TBH_gain * error;

    // taking back half crossed target
    if (sign(error) != sign(prev_error)) {
        if (first_cross) {
            output = lerp(tbh, output, first_cross_split);
            tbh = output;
            first_cross = false;
            printf("First cross\n");
        } else {
            // output = .5 * (output + tbh);
            output = lerp(tbh, output, .5);
            tbh = output;
        }
        prev_error = error;
    }

    if (lower != upper) {
        output = clamp(output, lower, upper);
    }
    return output;
}

double TakeBackHalf::get() { return output; }

void TakeBackHalf::set_limits(double low, double high) {

    lower = low;
    upper = high;
    printf("Set limits: %f, %f\n", lower, upper);
}

bool TakeBackHalf::is_on_target() { return fabs(prev_error) < on_target_threshhold; }

// Runs a Take Back Half variant to control RPM
// https://www.vexwiki.org/programming/controls_algorithms/tbh

// int spinRPMTask_TBH(void *wheelPointer)
// {
//     // Flywheel *wheel = (Flywheel *)wheelPointer;

//     double tbh = 0.0;
//     double output = 0.0;
//     double previous_error = 0.0;

//     while (true)
//     {
//         wheel->measure_RPM();

//         // reset if set to 0, this keeps the tbh val from screwing us up when we start up again
//         if (wheel->get_target() == 0)
//         {
//             output = 0;
//             tbh = 0;
//         }

//         double error = wheel->get_target() - wheel->getRPM();
//         output += wheel->getTBHGain() * error;
//         wheel->spin_raw(clamp(output, 0, 1), fwd);

//         if (sign(error) != sign(previous_error))
//         {
//             output = .5 * (output + tbh);
//             tbh = output;
//             previous_error = error;
//         }

//         vexDelay(1);
//     }

//     return 0;
// }