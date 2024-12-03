# Core
This is the host repository for the custom VEX libraries used by the RIT VEXU team

Automatically updated documentation is available at [here](https://rit-vex-u.github.io/Core/).
There is also a downloadable [reference manual](https://rit-vex-u.github.io/Core/refman.pdf).

![core_logo](https://github.com/RIT-VEX-U/Core/assets/12285261/e91c680b-5bf4-431c-b164-6631bef2a853)


## Getting Started

If you just want to start a project with Core, make a fork of the [Fork Template](https://github.com/RIT-VEX-U/ForkTemplate) and follow it's instructions.

To setup core for an existing project:
1. Create a new vex project (using the VSCode extension or other methods)
2. Initialize a git repository for the project
3. Execute `git subtree add --prefix=core git@github.com:RIT-VEX-U/Core.git main`
4. Update the vex Makefile (or any other build system) to know about the core files (`core/src` for source files, `core/include` for headers) (See [here](https://github.com/RIT-VEX-U/ForkTemplate/blob/a3f64236c0c98512b95327c833e8a8c05724bb7c/makefile#L15) for an example) 
5. Enable [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (Latest supported version is 3.4.0):
    - `mkdir vendor`
    - `git submodule add https://gitlab.com/libeigen/eigen.git vendor/eigen`
    - `cd vendor/eigen`
    - `git checkout 3.4.0`
    - Add the following to the `makefile` to give Core access to the library: `INC += -Ivendor/eigen` (See [here](https://github.com/RIT-VEX-U/ForkTemplate/blob/a3f64236c0c98512b95327c833e8a8c05724bb7c/makefile#L34) for an example)


If you only wish to use a single version of Core, you can simply clone core/ into your project and add the core source and header files to your makefile.


## Features
Here is the current feature list this repo provides:

Subsystems (See [Wiki/Subsystems](https://github.com/RIT-VEX-U/Core/wiki/2-%7C-Subsystems)):  
- Tank drivetrain (user control / autonomous)
- Mecanum drivetrain (user control / autonomous)
- Odometry
  - Tank (Differential)
  - [N-Pod](https://github.com/RIT-VEX-U/Core/commit/37daf076ac1b6dea6723724ad3061d502fea1b08)
- Flywheel
- Lift
- Custom encoders

Utilities (See [Wiki/Utilites](https://github.com/RIT-VEX-U/Core/wiki/3-%7C-Utilites)):  
- PID controller
- FeedForward controller
- Trapezoidal motion profile controller
- Pure Pursuit
- Generic auto program builder
- Auto program UI selector
- Mathematical classes (Vector2D, Moving Average)
