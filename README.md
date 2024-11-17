# Core
This is the host repository for the custom VEX libraries used by the RIT VEXU team

Automatically updated documentation is available at [here](https://rit-vex-u.github.io/Core/).
There is also a downloadable [reference manual](https://rit-vex-u.github.io/Core/refman.pdf).

![core_logo](https://github.com/RIT-VEX-U/Core/assets/12285261/e91c680b-5bf4-431c-b164-6631bef2a853)


## Getting Started

The recommended way to use this repo is through [git subtree](https://git-memo.readthedocs.io/en/latest/subtree.html).

To setup core for a new project
1. Create a new vex project (using the VSCode extension or other methods)
2. Initialize a git repository for the project
3. Execute `git subtree add --prefix=core git@github.com:RIT-VEX-U/Core.git main`
4. Update the vex Makefile (or any other build system) to know about the core files (`core/src` for source files, `core/include` for headers)
5. Start coding :)

Or, you can use the [template](https://github.com/RIT-VEX-U/RobotTemplate) from the RIT VEX U organization.

If you wish to contribute, follow the instructions at [Wiki/ProjectSetup](https://github.com/RIT-VEX-U/Core/wiki/1-%7C-Project-Setup)

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
