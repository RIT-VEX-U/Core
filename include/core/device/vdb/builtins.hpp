#pragma once
#include "core/device/vdb/types.hpp"
#include "core/subsystems/odometry/odometry_base.h"
#include <Eigen/Dense>
#include <memory>

#include "vex.h"
#include <string>
namespace VDP {
/**
 * Defines a record that holds a timestamp and data
 */
class TimestampedRecord : public Record {
  public:
    /**
     * Creates a record that contains a
     * Float of a timestamp
     * Part of data
     * @param name the name of the record to create
     * @param data the data to put into the record
     */
    TimestampedRecord(std::string name, Part *data);
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
class MotorDataRecord : public Record {
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
    MotorDataRecord(std::string name, vex::motor &mot);
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
 * Defines a record that holds odometry values to be sent to the board
 */
class OdometryDataRecord : public Record {
  public:
    /**
     * Creates a record that contains a
     * Float of the odometry X postion
     * Float of the odometry Y postion
     * Float of the odometry Rotation
     * @param name the name of the record to create
     * @param odom the odometry to get data from
     */
    OdometryDataRecord(std::string name, OdometryBase &odom);
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
 * Defines a record sets odometry values from the board
 */
class OdometryControlRecord : public Record {
  public:
    /**
     * Creates a record for taking odometry data from the debug board
     * @param name the name of the record to create
     * @param odom the odometry to get data from
     */
    OdometryControlRecord(std::string name, OdometryBase &odom);
    /**
     * sets the odom position to the values from the board
     */
    void response() override;

  private:
    OdometryBase &odom;

    std::shared_ptr<Float> X;
    std::shared_ptr<Float> Y;
    std::shared_ptr<Float> ROT;
};

/**
 * Defines a record that holds pid values to be sent to the board
 */
class PIDDataRecord : public Record {
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
    PIDDataRecord(std::string name, PID &pid);
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
/**
 * Defines a record for setting pid values from the board
 */
class PIDControlRecord : public Record {
  public:
    /**
     * Creates a record for setting pid values from the board
     * @param name the name of the record to create
     * @param pid the pid to get data from
     */
    PIDControlRecord(std::string name, PID &pid);
    /**
     * sets the PID values to the values from the board
     */
    void response() override;

  private:
    PID &pid;

    std::shared_ptr<Float> P;
    std::shared_ptr<Float> I;
    std::shared_ptr<Float> D;
};

/**
 * Defines a record for testing purposes, currently tests a float and int64
 */
class TestRecord : public Record {
  public:
    /**
     * Defines a record for testing purposes, currently tests a float and int64
=    */
    TestRecord(std::string name, double test_float, int64_t test_int64);
    void response() override;
    void fetch() override;

  private:
    double test_float;
    int64_t test_int64;

    std::shared_ptr<Float> test_float_ptr;
    std::shared_ptr<Int64> test_int64_ptr;
};
} // namespace VDP
