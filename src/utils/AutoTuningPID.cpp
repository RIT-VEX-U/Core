#include "vex.h"
#include <atomic>
#include <string>
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/AutoTuningPID.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/command_structure/command_controller.h"

#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/tank_drive.h"

vex::timer stopWatch;
double finalP;
double period;
bool finished = false;

//Basic method to calculate and use the PID values based on the given values from the setPIDDrive and setPIDTurn methods
//Uses various formulas from the Zeigler-Nichols Method
void findPID(std::string method, PID::pid_config_t &thePID){
    printf("BadP: %f BadI: %f BadD: %f\n", thePID.p, thePID.i, thePID.d);
    vexDelay(5);
    if(method == "P"){
        printf("Using P: ");
        vexDelay(5);
        thePID.p *= 0.5;
        vexDelay(5);
    }
    else if(method == "PI"){
        printf("Using PI: ");
        vexDelay(5);
        thePID.p *= 0.45;
        vexDelay(5);
        thePID.i = 0.83 * period;
        vexDelay(5);
    }
    else if(method == "PD"){
        printf("Using PD: ");
        vexDelay(5);
        thePID.p *= 0.8;
        vexDelay(5);
        thePID.d = 0.125 * period;
        vexDelay(5);
    }
    else if(method == "Classic PID"){
        printf("Using Classic PID: ");
        vexDelay(5);
        thePID.p *= 0.6;
        vexDelay(5);
        thePID.i = 0.5 * period;
        vexDelay(5);
        thePID.d = 0.125 * period;
        vexDelay(5);
    }
    else if(method == "Pessen Integral"){
        printf("Using Pessen Integral: ");
        vexDelay(5);
        thePID.p *= 0.7;
        vexDelay(5);
        thePID.i = 0.4 * period;
        vexDelay(5);
        thePID.d = 0.15 * period;
        vexDelay(5);
    }
    else if(method == "Some Overshoot"){
        printf("Using Some Overshoot: ");
        vexDelay(5);
        thePID.p *= (1.0 / 3.0);
        vexDelay(5);
        thePID.i = 0.5 * period;
        vexDelay(5);
        thePID.d = (1.0 / 3.0) * period;
        vexDelay(5);
    }
    else if(method == "No Overshoot"){
        printf("Using No Overshoot: ");
        thePID.p *= 0.2;
        thePID.i = 0.5 * period;
        thePID.d = (1.0 / 3.0) * period;
    }
    vexDelay(5);
    printf("GoodP: %f GoodI: %f GoodD: %f\n", thePID.p, thePID.i, thePID.d);
};

//method to calculate the PID values for driving a robot
class setPIDDrive : public AutoCommand{
    public:
    bool isOscillating = false;
    double finalP;
    int initTime;
    double foundInitVelocity = false;
    bool initVelocityPos = true;
    bool isVelocityPos = true;
    PID::pid_config_t driveConfig;
    vex::motor_group rightMotors;
    //Input the driveConfig and right motors
    setPIDDrive(PID::pid_config_t *driveConfig, vex::motor_group *rightMotors) : driveConfig(*driveConfig), rightMotors(*rightMotors){

    }
    bool run() override{
    //resets pid values to 0
    driveConfig.p = 0;
    driveConfig.i = 0;
    driveConfig.d = 0;
    //resets stopwatch
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());
    //finds whether the bot is going forwards or backwards at the start of tuning
    //does this by increasing p until it gets a velocity over or under 0 and sets the initial velocity
    while(!foundInitVelocity){
        vexDelay(1);
        printf("Velocity: %f, P: %f \n", rightMotors.velocity(percent), driveConfig.p);
        if((rightMotors.velocity(percent) > 0.01 || rightMotors.velocity(percent) < -0.01)){
            printf("found init velocity\n");
            initVelocityPos = (rightMotors.velocity(percent) > 0);
            foundInitVelocity = true;
            vexDelay(1);
        }
        else if((rightMotors.velocity(percent) < 0.01 && rightMotors.velocity(percent) > -0.01)){
            driveConfig.p += 0.0005;
            vexDelay(1);
        }
    }
    //while loop for getting the PID values
    while(!finished){
        vexDelay(1);
        //checks whether the current velocity is positive or not
        if((-0.01 >= rightMotors.velocity(percent) || rightMotors.velocity(percent) >= 0.01)){
        isVelocityPos = (rightMotors.velocity(percent) > 0);
        vexDelay(1);
        }

        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);
        if(isOscillating){
            printf("Oscillating...");
        }

        printf("Velocity: %f \n", rightMotors.velocity(percent));
        vexDelay(1);
        //increases the P value until the velocity sign has changed
        if((initVelocityPos == isVelocityPos) && !isOscillating && foundInitVelocity){
            driveConfig.p += 0.003;
            printf("P: %f ", driveConfig.p);
            vexDelay(1);
        }
        //once the sign has changed, get the time for the start of the period of oscillation,
        //increase p a little to get a stable oscillation, and stop increasing it
        else if((initVelocityPos != isVelocityPos) && !isOscillating){
            vexDelay(1);
            isOscillating = true;
            printf(" Oscillation Started\n");
            vexDelay(1);
            finalP = driveConfig.p;
            isOscillating = true;
            driveConfig.p *= 1.4;
            initTime = stopWatch.time();
            printf("Pfinal: %f", finalP);
            vexDelay(1);
        }
        //after the oscillation has started and the velocity sign has changed once again, half the oscillation period is over,
        //get the period by multiplying it by 2 and dividing it by 1000 because the timer uses milliseconds
        //call the findPID values and give it the period and the driveConfig which contains the current P value
        else if((initVelocityPos == isVelocityPos) && isOscillating){
            printf("Unchanged P: %f", driveConfig.p);
            printf("InitTime: %d\n", initTime);
            printf("Final Time: %d ", stopWatch.time());
            period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
            printf("Period: %f\n", period);
            vexDelay(1);
            findPID("Classic PID", driveConfig);
            finished = true;
            vexDelay(1);
            }
        vexDelay(1);
    }
return true;
}
};
//Basic command to print the error to run in parralell with a robot driving or turning autocommand
class checkError: public AutoCommand{
    public:
    double target;
    bool driveTurn;
    OdometryTank odom;
    checkError(double target, bool driveTurn, OdometryTank &odom) : target(target), driveTurn(driveTurn), odom(odom){

    }
    bool run() override{
        while(true){
            vexDelay(5);
            if(driveTurn){
            printf("Error: %f\n", target - odom.get_position().x);
            }
            else{
                printf("Error: %f\n", target - odom.get_position().rot);
            }
        }
        return true;
    }

};
//Conditoon used in the setPID commands to check whether or not the commands have finished
class finishedCond : public Condition {
    bool test() override {
        return finished;
    }
};
//Very WIP setPIDTurn, currently does not work, may try to fix later
class setPIDTurn : public AutoCommand{
    public:
    bool isOscillating = false;
    double finalP;
    int initTime;
    double foundInitVelocity = false;
    bool initVelocityPos = true;
    bool isVelocityPos = true;
    PID::pid_config_t &turnConfig;
    vex::inertial imu;
    setPIDTurn(PID::pid_config_t &turnConfig, vex::inertial &imu) : turnConfig(turnConfig), imu(imu){};
    bool run() override{
    turnConfig.p = 0;
    turnConfig.i = 0;
    turnConfig.d = 0;
    
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());

    while(!foundInitVelocity){
        vexDelay(1);
        printf("Velocity: %f, P: %f \n", imu.gyroRate(zaxis, rpm), turnConfig.p);
        if((imu.gyroRate(zaxis, rpm) > 0.01 || imu.gyroRate(zaxis, rpm) < -0.01)){
            printf("found init velocity\n");
            initVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
            foundInitVelocity = true;
            vexDelay(1);
        }
        else if((imu.gyroRate(zaxis, rpm) < 0.01 && imu.gyroRate(zaxis, rpm) > -0.01)){
            turnConfig.p += 0.0005;
            vexDelay(1);
        }
    }
    while(!finished){
        vexDelay(1);
        printf("Time: %d ", stopWatch.time());

        if((-0.01 >= imu.gyroRate(zaxis, rpm) || imu.gyroRate(zaxis, rpm) >= 0.01)){
        isVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
        vexDelay(1);
        }

        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);

        if(isOscillating){
            printf("Oscillating...");
        }

        printf("Velocity: %f \n", imu.gyroRate(zaxis, rpm));
        vexDelay(1);
        if((initVelocityPos == isVelocityPos) && !isOscillating && foundInitVelocity){
            turnConfig.p += 0.002;
            printf("P: %f", turnConfig.p);
            vexDelay(1);
        }
        else if((initVelocityPos != isVelocityPos) && !isOscillating){
            vexDelay(1);
            isOscillating = true;
            printf(" Oscillation Started\n");
            vexDelay(1);
            finalP = turnConfig.p;
            isOscillating = true;
            turnConfig.p *= 1.5;
            initTime = stopWatch.time();
            printf("Pfinal: %f", finalP);
            vexDelay(1);
        }
        else if((initVelocityPos == isVelocityPos) && isOscillating){
            printf("Unchanged P: %f", turnConfig.p);
            printf("InitTime: %d\n", initTime);
            printf("Final Time: %d ", stopWatch.time());
            period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
            printf("Period: %f\n", period);
            vexDelay(1);
            finished = true;
            vexDelay(1);
            }
        vexDelay(5);
    }
    printf("setPID Done");
    return true;
    }
};

//voids to run when pressing a button to tune the drivePID and drive forward while checking the Error
void AutoTuningTools::TuneDrivePID(){
    for(int i = 100; i > 0;  i--){
            printf("\n");
        }
        vexDelay(10);
        CommandController cc{
            cfg.odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                driveSys.DriveForwardCmd(cfg.distance, cfg.dir, cfg.startspeed, cfg.endspeed)->withCancelCondition(new finishedCond()),
                new setPIDDrive(cfg.pidConfig, cfg.right_motors)
            }
        };
        
        cc.run();
};
void AutoTuningTools::driveWithError(){
        CommandController cc{
            cfg.odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                driveSys.DriveForwardCmd(cfg.distance, cfg.dir, cfg.startspeed, cfg.endspeed)->withCancelCondition(new finishedCond()),
                new checkError(24, true, cfg.odom)
            }
        };
        
        cc.run();
};