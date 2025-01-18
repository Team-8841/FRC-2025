// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO: Update for robot
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ElevatorConstants
  {
    public static final int M1_CANID = 16; //Left Motor for elevator
    public static final int M2_CANID = 17; // Right Motor for elevator

    public static final int CURRENT_LIMIT = 40;

    // Feedforward
    public static final double FF_kS = 0.5;  // Static gain (N·m)
    public static final double FF_kG = 1.0;  // Gravity gain (N·m)
    public static final double FF_kV = 0.1;  // Velocity gain (N·m/(m/s))
    public static final double FF_kA = 0.05; // Acceleration gain (N·m/(m/s^2))

    //PID 
    public static final double PID_kS = 0.1;
    public static final double PID_kG = 0.0;
    public static final double PID_kA = 0.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
      
  // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }



  public static final class GripperConstants { //TODO: change this one we figure out the canivore
    public static final int WRIST_MOTOR_CANID = 3;
    public static final int GRIPPER_MOTOR1_CANID = 4;
    public static final int GRIPPER_MOTOR2_CANID = 5;

    public static final double WRIST_ALLOWED_ERROR = 200; //TODO: Tune this value
    
    public static final double WRIST_P = 1;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0;

    public static final int CORAL_SENSOR_PORT = 0; //TODO: Change this value
    public static final int ALGAE_SENSOR_PORT = 1; //TODO: Change this value
    
  }

  public static final class LightingConstants { 
    public static final int CANDLE_CANID = 0; //TODO: Change this value
  }
}
