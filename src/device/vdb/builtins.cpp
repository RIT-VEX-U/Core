#include "core/device/vdb/builtins.hpp"

#include "core/device/vdb/protocol.hpp"
#include "core/device/vdb/types.hpp"

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
TimestampedRecord::TimestampedRecord(std::string name, Part *data)
    : Record(name), timestamp(new Float("timestamp", []() { return (float)vexSystemTimeGet() / 1000; })), data(data) {
    Record::setFields({timestamp, (PartPtr)data});
}
/**
 * sets the data that the Timestamp Parts hold
 */
void TimestampedRecord::fetch() {
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
MotorDataRecord::MotorDataRecord(std::string name, vex::motor &motor)
    : Record(std::move(name)), mot(motor), pos(new Float("Position(deg)")), vel(new Float("velocity(dps)")),
      temp(new Float("Temperature(C)")), voltage(new Float("Voltage(V)")), current(new Float("Current(%)")) {
    Record::setFields({pos, vel, temp, voltage, current});
}
/**
 * sets the data that the Motor Parts hold
 */
void MotorDataRecord::fetch() {
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
OdometryDataRecord::OdometryDataRecord(std::string name, OdometryBase &odom)
    : Record(std::move(name)), odom(odom), X(new Float("X")), Y(new Float("Y")), ROT(new Float("Rotation")) {
    Record::setFields({X, Y, ROT});
}
/**
 * sets the data that the Odometry Parts hold
 */
void OdometryDataRecord::fetch() {
    X->setValue((float)odom.get_position().x());
    Y->setValue((float)odom.get_position().y());
    ROT->setValue((float)odom.get_position().rotation().degrees());
}
/**
 * Creates a record for taking odometry data from the debug board
 * @param name the name of the record to create
 * @param odom the odometry to get data from
 */
OdometryControlRecord::OdometryControlRecord(std::string name, OdometryBase &odom)
    : Record(std::move(name)), odom(odom), X(new Float("X")), Y(new Float("Y")), ROT(new Float("Rotation")) {
    Record::setFields({X, Y, ROT});
}
/**
 * sets the odometry position to the values from the debug board
 */
void OdometryControlRecord::receive(VDP::Packet &pac) {
    PacketReader reader(pac);
    X.get()->read_data_from_message(reader);
    Y.get()->read_data_from_message(reader);
    ROT.get()->read_data_from_message(reader);
    odom.set_position({X->getValue(), Y->getValue(), ROT->getValue()});
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
PIDDataRecord::PIDDataRecord(std::string name, PID &pid)
    : Record(std::move(name)), pid(pid), P(new Float("P")), I(new Float("I")), D(new Float("D")),
      ERROR(new Float("Error")), OUTPUT(new Float("Output")), TYPE(new String("Type")) {
    Record::setFields({TYPE, P, I, D, ERROR, OUTPUT});
}
/**
 * sets the data that the PID Parts hold to be sent to the board
 */
void PIDDataRecord::fetch() {
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

PIDControlRecord::PIDControlRecord(std::string name, PID &pid)
    : Record(std::move(name)), pid(pid), P(new Float("P")), I(new Float("I")), D(new Float("D")) {
    Record::setFields({P, I, D});
}

void PIDControlRecord::receive(VDP::Packet &pac) {
    PacketReader reader(pac);
    P->read_data_from_message(reader);
    I->read_data_from_message(reader);
    D->read_data_from_message(reader);
    pid.config.p = P->getValue();
    pid.config.i = I->getValue();
    pid.config.d = D->getValue();
}

} // namespace VDP
