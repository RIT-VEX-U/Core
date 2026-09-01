#pragma once

#include <functional>
#include <random>
#include <string>
#include <vector>
#include "vex.h"

/// Creates an Initialization
#define INIT(name, func, meta) Initialization{name, func, meta}

/// Creates a purely-functional Initialization, with a blank name and 1 for its metadata
#define FUNC_INIT(func) Initialization{"", func, 1}

// Creates an Initialization with a set-able metadata for weighted operations
#define WEIGHTED_INIT(func, weight) Initialization{"", func, weight}

/// Creates an Initialization whose metadata allows it to appear RED when using InitializerScreen
#define RED_INIT(name, func) Initialization{name, func, ClrRed }

/// Creates an Initialization whose metadata allows it to appear BLUE when using InitializerScreen
#define BLUE_INIT(name, func) Initialization{name, func, ClrBlue}

/// Creates an Initialization whose metadata allows it to appear GRAY when using InitializerScreen
#define NEUTRAL_INIT(name, func) Initialization{name, func, ClrGray}

/**
 * Initialization is a helper structure of the Initializer class used to group an initialization function with relevant metadata.
 */
struct Initialization {
    const std::string name;           /// The name of the Initialization
    const std::function<void()> init; /// The function that executes when this Initialization is selected
    const unsigned int meta;          /// Additional metadata that may be needed by the Initializer selector function
};


/**
 * A namespace containing predefined selector functions and wrappers that can be passed to an Initializer.
 */
namespace Selector {
    
/// The type of an Initializer's Selector function
using selector_t = size_t();

/// A constant index that is used in selector functions to represent when a selection has not yet been made
constexpr size_t NO_SELECTION_INDEX = SIZE_MAX;

/**
 * Returns a selector function that selects the Initialization corresponding to N
 * @param n     The Initialization to select
 * @returns     The value of N
 */
inline std::function<selector_t> select(size_t n) {
    return [=]() {
        return n;
    };
}

/**
 * Returns a selector function that selects a random Initialization
 * @pre         Call srand() in pre-initialization using the VEX Brain's high-resolution timer
 * @param a     The inclusive beginning of the random range
 * @param b     The exclusive end of the random range
 * @returns     A random size_t value in the range [A,B). If the range is invalid (B<A), then this will return 0.
 */
inline std::function<selector_t> random(size_t a, size_t b) {
    return [=]() {
        if(b < a) return (size_t) 0;
        if(a == b) return a;
        return a + ((size_t) rand()) % (b - a);
    };
}

/**
 * Returns a selector function that selects a random Initialization
 * @pre     Call srand() in pre-initialization using the VEX Brain's high-resolution timer
 * @param n The exclusive end of the random range
 * @returns A random size_t value in the range [0,N).
 */
inline std::function<selector_t> random(size_t n) {
    return random(0, n);
}

/**
 * Returns a selector function wrapper that allows selection to be skipped based on a compile-time flag.
 * This should only be used for debugging, as it would otherwise defeat the purpose of the Initializer.
 * @tparam FLAG     The Initialization to select if compile-time selection is wanted, or nothing to select Initializations based on the provided selector.
 * @param selector  The selector function for runtime selection
 * @returns         The compile selector function if flag is defined, and the selector argument if flag is not defined
 */
template<size_t FLAG = NO_SELECTION_INDEX>
inline std::function<selector_t> skippable(std::function<selector_t> selector) {
    return (FLAG == NO_SELECTION_INDEX && selector) ? selector : select(FLAG);
}

/**
 * A selector function wrapper that allows a selection to timeout, returning the fallback if it does.
 * @pre             The selector function must be thread-safe
 * @param selector  The thread-safe selector function to execute
 * @param microsec  The amount of time until a timeout occurs, in microseconds
 * @param fallback  The return value if a timeout occurs
 * @param cancel    The callback used to notify the rest of the program if a timeout occured. It will always be passed the fallback value as its argument.  
 * @returns         The return value of selector, or fallback if a timeout occurs
 */
inline std::function<selector_t> timeout(std::function<selector_t> selector, uint64_t microsec, unsigned int fallback = NO_SELECTION_INDEX, std::function<void(size_t)> cancel = nullptr) {
    if(!selector) return [=]() { return fallback; };
    struct thread_args {
        std::function<selector_t> selector;
        size_t output;
    };
    return [=]() {
        vex::timer timer;
        uint64_t thread_start = timer.systemHighResolution();
        thread_args targs = {selector, NO_SELECTION_INDEX};
        
        vex::thread sThread([](void* args) {
            ((thread_args*) args)->output = ((thread_args*) args)->selector();
        }, &targs);
        
        while(sThread.joinable()) {
            if(timer.systemHighResolution() - thread_start > microsec) sThread.interrupt();
            else if(targs.output == NO_SELECTION_INDEX) {
                vexDelay(100);
                continue;
            }

            sThread.join();
        }

        if(targs.output == NO_SELECTION_INDEX) {
            cancel(fallback);
            return fallback;
        } else return targs.output;
    };
}

/**
 * A selector function wrapper that delays the program for a set amount of time before selecting the Initialization.
 * @param selector  The selector function to execute
 * @param ms        The amount of time to delay for, in milliseconds
 * @returns         The return value of selector
 */
inline std::function<selector_t> delayed(std::function<selector_t> selector, unsigned int ms) {
    return [=]() {
        vexDelay(ms);
        return selector();
    };
}

/**
 * A selector function that utilizes a potentiometer to select the Initialization
 * Each initialization is corresponds to an equal percentage of the potentiometer's range.
 * @param potentiometer     The potentiometer to select inputs based on
 * @param initializations   The number of initializations
 * @returns                 A selector function that selects its Initialization based utilizing a potentiometer
 */
inline std::function<selector_t> potentiometer(vex::pot& potentiometer, const size_t& initializations) {
    return [=, &potentiometer]() {
        vexDelay(250);
        double region = 100.0 / initializations;
        double angle = potentiometer.value(vex::percentUnits::pct);
        return (size_t) ((angle < 100) ? angle : 99.9) / region;
    };
}

/**
 * A selector function that utilizes a potentiometer to select the Initialization.
 * This function uses the metadata of the Initializations to determine their representation on the potentiometer.
 * @param potentiometer     The potentiometer to select inputs based on
 * @param initializations   A vector of initializations
 * @returns                 A selector function that selects its Initialization based utilizing a potentiometer
 */
inline std::function<selector_t> weighted_potentiometer(vex::pot& potentiometer, std::vector<Initialization>& initializations) {
    return [&]() {
        vexDelay(250);
        size_t total_weights = 0;
        for(Initialization initialization : initializations) total_weights += initialization.meta;

        size_t angle = (potentiometer.value(vex::percentUnits::pct)/100.0)*total_weights;
        for(int i = initializations.size()-1; i > 0; i--) {
            if(initializations.at(i).meta == 0) continue;

            total_weights -= initializations.at(i).meta;
            if(angle >= total_weights) return i;
        }
        return 0;
    };
}

}; // namespace Selector


/**
 * Initializer is a utility that allows one program to call different robot initializations depending on a selection function, effectively reducing the amount of program slots that need to be redownloaded when making modifications to the entire code.
 */
class Initializer {
public:
    /**
     * Shorthand for constructing an Initializer that bypasses selection and simply calls the provided callback function during initialization. Ideally, this constructor will never be used, but is included to allow quick, simple changes to programs without forcing extra changes or unnecessary syntax.
     * @param initialize        The callback function used to initialize the robot
     */
    Initializer(std::function<void()> initialize);

    /**
     * The primary constructor for Initializer
     * @param initializations   A vector of Initializations
     * @param selector          A function used to select the Initialization. If a nullptr is passed for this, the initialize() function will call the first Initialization.
     * @param pre_init          A function for common code that runs before the selection-dependent initialization. This should only be used to setup anything needed by the initializer selector function.
     * @param post_init         A function for common code that runs after the selection-dependent initialization
     */
    Initializer(std::vector<Initialization> initializations, std::function<Selector::selector_t> selector,
                std::function<void()> pre_init = nullptr, std::function<void()> post_init = nullptr);

    /**
     * Initializes the robot
     */
    void initialize();

    /**
     * Returns the vector storing the Initializations
     * @returns A constant reference to the internal vector that stores Initializations
     */
    const std::vector<Initialization>& initializations() const;

    /**
     * Returns the number of Initializations that can be selected
     * @returns The length of the internal collect of Initializations
     */
    const std::size_t initialization_count() const;

    /**
     * Returns the name of the selected Initialization
     * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's name. Otherwise, this returns an empty string.
     */
    const std::string selected_name() const;

    /**
     * Returns the meta of the selected Initialization
     * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's meta. Otherwise, this returns SIZE_MAX.
     */
    unsigned int selected_meta() const;

    /**
     * Returns the index of the selected Initialization
     * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's index. Otherwise, this returns Selector::NO_SELECTION_INDEX.
     */
    unsigned int selected_index() const;

    /**
     * Returns whether or not the robot is currently uninitialized
     * @returns Returns true if the robot has not been initialized, and false if it already has.
     */
    bool uninitialized() const;

private:
    std::function<Selector::selector_t> selector;
    std::vector<Initialization> initialization_list;

    std::function<void()> pre_init;
    std::function<void()> post_init;

    Initialization *selected_initialization = nullptr;
    size_t selection = Selector::NO_SELECTION_INDEX;
    bool initialized = false;
};