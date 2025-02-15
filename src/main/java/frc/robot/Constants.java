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
    public static final String CANBUS_NAME = "canivore";
    public static final int M1_CANID = 18; //Left Motor for elevator
    public static final int M2_CANID = 16; // Right Motor for elevator

    public static final int DIO_TOPSENSOR = 1;
    public static final int DIO_BOTTOMSENSOR = 0;

    public static final int CURRENT_LIMIT = 60;

    public static final int TRAVEL_ROT = 10; // Number of rotations to travel full distance

    public static final double RAMP_UP = .1;

    //PID 
    public static final double PID_P = 0.4;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.01;
    public static final double PID_FF = .000156;

    // Predefined Positions
    // 98 IS ABSOLUTE TOP
    //public static final double TOP_POS = 95;
    //public static final double HIGH_POS = 69;
    //public static final double MID_POS = 38.5;
    //public static final double LOW_POS = 14.8;
    //public static final double BOTTOM_POS = 5.0;

    public static final double MAX_POS = 98.0;
    public static final double MIN_POS = 0.0;

  }

  public static final class SetpointConstants { //TODO: Update these values
    // Positions for each elevator and wrist position
    // Setup as an array of {Elevator Position, Wrist Position}
    public static final double[] CoralL1 = {14.8, 10}; // 14.8
    public static final double[] CoralL2 = {38.5, 20}; // 38.5
    public static final double[] CoralL3 = {69, 30}; //69
    public static final double[] CoralL4 = {95, 40}; //95

    public static final double[] AlgaeL1 = {14.8, 10};
    public static final double[] AlgaeL2 = {38.5, 20};
    public static final double[] AlgaeL3 = {69, 30};
    public static final double[] AlgaeL4 = {195, 40};
    

    public static final double[] startingConfiguration = {0, 0};
    public static final double[] groundPickup = {5, 10};
    public static final double[] feederStation = {20, 0};
  
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;
      
  // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;




    public static final int CoralL1 = 1; //Button 1
    public static final int CoralL2 = 2; //Button 2
    public static final int CoralL3 = 3; //Button 3
    public static final int CoralL4 = 4; //Button 4

    public static final int AlgaeL1 = 5; //Button 5
    public static final int AlgaeL2 = 6; //Button 6
    public static final int AlgaeL3 = 7; //Button 7
    public static final int AlgaeL4 = 8; //Button 8

    public static final int groundPickup = 9;
    public static final int feederStation = 10;

    public static final int IntakeIn = 11;
    public static final int IntakeOut = 12;

    public static final int ElevatorLock = 13;
    public static final int ManualOverride = 14;
  }



  public static final class GripperConstants { //TODO: change this one we figure out the canivore
    public static final String CANBUS_NAME = "canivore";
    public static final int WRIST_MOTOR_CANID = 19;
    public static final int GRIPPER_MOTOR1_CANID = 20;
    public static final int HOME_SENSOR_PORT = 5;
    public static final int ROT_SENSOR_PORT = 6;

    public static final double WRIST_ALLOWED_ERROR = 5; //TODO: Tune this value
    
    public static final double WRIST_P = 0.1;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0.01;

    public static final double RAMP_UP = 0.05;

    public static final int CORAL_SENSOR_PORT = 2; //TODO: Change this value
    public static final int ALGAE_SENSOR_PORT = 3; //TODO: Change this value

    // 0 is Home (UP) and -86 is Down rotated clockwise as viewed from right side 
    public static final double CLOSE_POS = -20.0;
    public static final double MID_POS = -40.0;
    public static final double FAR_POS = -80.0;

    public static final int MIN_POS = 0;
    public static final int MAX_POS = -86;

   
    
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MAIN_CANID = 21;
    public static final int CLIMBER_FOLLOWER_CANID = 22;
    public static final String CANDLE_BUS = "canivore";
  }

  public static final class LightingConstants { 
    public static final int CANDLE_CANID = 23; //TODO: Change this value
    public static final String CANDLE_BUS = "rio";
  }
}
