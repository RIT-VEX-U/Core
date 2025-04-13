#include "../core/include/device/vdb/builtins.hpp"

#include "../core/include/device/vdb/protocol.hpp"
#include "../core/include/device/vdb/types.hpp"

#include "vex_motor.h"
#include "vex_units.h"
#include <cstdint>
#include <string>
#include <utility>

namespace VDP {
/**
 * Creates a record that contains a
 * Float of a timestamp
 * Part of data
 * @param name the name of the record to create
 * @param data the data to put into the record
 */
Timestamped::Timestamped(std::string name, Part *data)
    : Record(name), timestamp(new Float("timestamp", []() { return (float)vexSystemTimeGet() / 1000; })), data(data) {
    Record::setFields({timestamp, (PartPtr)data});
}
/**
 * sets the data that the Timestamp Parts hold
 */
void Timestamped::fetch() {
    timestamp->fetch();
    data->fetch();
}
/**
 * Creates a record that contains a
 * Float of the motor position
 * Float of the motor velocity
 * Float of the motor tempurature
 * Float of the motor voltage
 * Float of the motor current
 * @param name the name of the record to create
 * @param mot the motor to get data from
 */
Motor::Motor(std::string name, vex::motor &motor)
    : Record(std::move(name)), mot(motor), pos(new Float("Position(deg)")), vel(new Float("velocity(dps)")),
      temp(new Float("Temperature(C)")), voltage(new Float("Voltage(V)")), current(new Float("Current(%)")) {
    Record::setFields({pos, vel, temp, voltage, current});
}
/**
 * sets the data that the Motor Parts hold
 */
void Motor::fetch() {
    pos->setValue((float)mot.position(vex::rotationUnits::deg));
    vel->setValue((float)mot.velocity(vex::velocityUnits::dps));
    temp->setValue((float)mot.temperature(vex::temperatureUnits::celsius));
    voltage->setValue((float)mot.voltage(vex::voltageUnits::volt));
    current->setValue((float)mot.current(vex::percentUnits::pct));
}
/**
 * Creates a record that contains a
 * Float of the odometry X postion
 * Float of the odometry Y postion
 * Float of the odometry Rotation
 * @param name the name of the record to create
 * @param odom the odometry to get data from
 */
Odometry::Odometry(std::string name, OdometryBase &odom)
    : Record(std::move(name)), odom(odom), X(new Float("X")), Y(new Float("Y")), ROT(new Float("Rotation")) {
    Record::setFields({X, Y, ROT});
}
/**
 * sets the data that the Odometry Parts hold
 */
void Odometry::fetch() {
    X->setValue((float)odom.get_position().x());
    Y->setValue((float)odom.get_position().y());
    ROT->setValue((float)odom.get_position().rotation().degrees());
}
/**
 * Creates a record that contains a
 * Float of the pid P value
 * Float of the pid I value
 * Float of the pid D value
 * Float of the pid error
 * Float of the pid output
 * String of the pid type (linear or angular)
 * @param name the name of the record to create
 * @param pid the pid to get data from
 */
PIDRecord::PIDRecord(std::string name, PID &pid)
    : Record(std::move(name)), pid(pid), P(new Float("P")), I(new Float("I")), D(new Float("D")),
      ERROR(new Float("Error")), OUTPUT(new Float("Output")), TYPE(new String("Type")) {
    Record::setFields({TYPE, P, I, D, ERROR, OUTPUT});
}
/**
 * sets the data that the PID Parts hold
 */
void PIDRecord::fetch() {
    P->setValue((float)pid.config.p);
    I->setValue((float)pid.config.i);
    D->setValue((float)pid.config.d);
    ERROR->setValue((float)pid.get_error());
    OUTPUT->setValue((float)pid.get_output());
    if (pid.config.error_method == PID::ANGULAR) {
        TYPE->setValue("Angular");
    } else {
        TYPE->setValue("Linear");
    }
}

} // namespace VDP
