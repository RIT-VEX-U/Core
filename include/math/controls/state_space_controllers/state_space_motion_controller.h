// this will be similar to normal motion controller but will use lqr + lpiff for a 1d syste
// won't be suitable for a drivetrain 2d control, but will be suitable for single joints, elevators
// or 1d drivetrain control (fine for rotation and straight line control I guess?) for 2d it'll just be a linearized
// lqr like wpilib LTVDifferentialDriveController... I'm still not quite sure whether i'm good with this though
//
// before I make this I gotta make some file that just gives you preconfigured linear systems
// so like you say FlywheelSystem(kv, ka) and it gives you a LinearSystem