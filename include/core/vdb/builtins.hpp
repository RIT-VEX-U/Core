#pragma once

#include "../core/include/device/vdb/types.hpp"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include <memory>


#include "vex.h"
namespace VDP {
/**
 * Defines a record that holds a timestamp and data
 */
class Timestamped : public Record {
  public:
    /**
     * Creates a record that contains a
     * Float of a timestamp
     * Part of data
     * @param name the name of the record to create
     * @param data the data to put into the record
     */
    Timestamped(std::string name, Part *data);
    /**
     * sets the data that the Timestamp Parts hold
     */
    void fetch();

  private:
    std::shared_ptr<Float> timestamp;
    PartPtr data;
};
/**
 * Defines a record that holds motor values
 */
class Motor : public Record {
  public:
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
    Motor(std::string name, vex::motor &mot);
    /**
     * sets the data that the Motor Parts hold
     */
    void fetch() override;

  private:
    vex::motor &mot;

    std::shared_ptr<Float> pos;
    std::shared_ptr<Float> vel;
    std::shared_ptr<Float> temp;
    std::shared_ptr<Float> voltage;
    std::shared_ptr<Float> current;
};
/**
 * Defines a record that holds odometry values
 */
class Odometry : public Record {
  public:
    /**
     * Creates a record that contains a
     * Float of the odometry X postion
     * Float of the odometry Y postion
     * Float of the odometry Rotation
     * @param name the name of the record to create
     * @param odom the odometry to get data from
     */
    Odometry(std::string name, OdometryBase &odom);
    /**
     * sets the data that the Odometry Parts hold
     */
    void fetch() override;

  private:
    OdometryBase &odom;

    std::shared_ptr<Float> X;
    std::shared_ptr<Float> Y;
    std::shared_ptr<Float> ROT;
};
/**
 * Defines a record that holds pid values
 */
class PIDRecord : public Record {
  public:
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
    PIDRecord(std::string name, PID &pid);
    /**
     * sets the data that the PID Parts hold
     */
    void fetch() override;

  private:
    PID &pid;

    std::shared_ptr<Float> P;
    std::shared_ptr<Float> I;
    std::shared_ptr<Float> D;
    std::shared_ptr<Float> ERROR;
    std::shared_ptr<Float> OUTPUT;
    std::shared_ptr<String> TYPE;
};
} // namespace VDP
