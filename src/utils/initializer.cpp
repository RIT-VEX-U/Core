#include "core/utils/initializer.h"


/**
 * Shorthand for constructing an Initializer that bypasses selection and simply calls the provided callback function during initialization. Ideally, this constructor will never be used, but is included to allow quick, simple changes to programs without forcing extra changes or unnecessary syntax.
 * @param initialize        The callback function used to initialize the robot
 */
Initializer::Initializer(std::function<void()> initialize)
 : selector(nullptr), initialization_list({}), pre_init(initialize), post_init(nullptr) {}

/**
 * The constructor for Initializer
 * @param initializations   A vector of Initializations
 * @param selector          A function used to select the Initialization. If a nullptr is passed for this, the initialize() function will call the first Initialization.
 * @param pre_init          A function for common code that runs before the selection-dependent initialization. This should only be used to setup anything needed by the initializer selector function.
 * @param post_init         A function for common code that runs after the selection-dependent initialization
 */
Initializer::Initializer(   std::vector<Initialization> initializations, std::function<Selector::selector_t> selector,
                            std::function<void()> pre_init, std::function<void()> post_init )
 : selector(selector), initialization_list(initializations), pre_init(pre_init), post_init(post_init) {}

/**
 * Initializes the robot
 */
void Initializer::initialize() {
    if(this->pre_init) this->pre_init();

    if(this->selector) this->selection = this->selector();
    if(this->selection < this->initialization_list.size()) 
        (this->selected_initialization = &this->initialization_list.at(this->selection))->init();

    if(this->post_init) this->post_init();
    this->initialized = true;
}

/**
 * Returns the vector storing the Initializations
 * @returns A constant reference to the internal vector that stores Initializations
 */
const std::vector<Initialization>& Initializer::initializations() const {
    return this->initialization_list;
}

/**
 * Returns the number of Initializations that can be selected
 * @returns The length of the internal collect of Initializations
 */
const std::size_t Initializer::initialization_count() const {
    return this->initialization_list.size();
}

/**
 * Returns the name of the selected Initialization
 * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's name. Otherwise, this returns an empty string.
 */
const std::string Initializer::selected_name() const {
    return (this->selected_initialization == nullptr) ? "" : this->selected_initialization->name;
}

/**
 * Returns the meta of the selected Initialization
 * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's meta. Otherwise, this returns 0xFFFFFFFF.
 */
unsigned int Initializer::selected_meta() const {
    return (this->selected_initialization == nullptr) ? SIZE_MAX : this->selected_initialization->meta;
}

/**
 * Returns the index of the selected Initialization
 * @returns If an Initialization has been selected using the initialize() function, this returns that Initialization's index. Otherwise, this returns 0xFFFFFFFF.
 */
unsigned int Initializer::selected_index() const {
    return this->selection;
}

/**
 * Returns whether or not the robot is currently uninitialized
 * @returns Returns true if the robot has not been initialized, and false if it already has.
 */
bool Initializer::uninitialized() const {
    return !this->initialized;
}