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

    public static final double RAMP_UP = .3;

    //PID 
    public static final double PID_P = 0.385;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.056; //.058
    public static final double PID_FF = .000156;

    // Predefined Positions
    // 58 IS ABSOLUTE TOP

    public static final double MAX_POS = 58.0;
    public static final double MIN_POS = 0.0;


    public static final double ELEVATOR_TOLERANCE = 2.0;

  }

  public static final class SetpointConstants { //TODO: Update these values
    // Positions for each elevator and wrist position
    // Setup as an array of {Elevator Position, Wrist Position} 
    // Absolute Max elevator position is 58 DO NOT go above this value Try to keep it 1 less than max hight
    // Absolute Max wrist position is 100 DO NOT go above this number 

    public static final double[] CoralL1 = {1, 79};  //12, 31 Reef L1
    public static final double[] CoralL2 = {11, 72};  //14, 44 Reef L2 
    public static final double[] CoralL3 = {26, 72};  //28, 44 Reef L3
    public static final double[] CoralL4 = {54, 68};  //54, 67 Reef L4

    public static final double[] AlgaeL1 = {1, 67};   // 1, 36    processing station
    public static final double[] AlgaeL2 = {14, 77};  //37, 36    Reef Algae L2
    public static final double[] AlgaeL3 = {28, 77};  //61, 36    Reef Algae L3
    public static final double[] AlgaeL4 = {57, 101};  //95, 20    Barge Shot
    

    public static final double[] startingConfiguration = {1, 0};
    public static final double[] startingHomeConfiguration = {1,0}; 
    public static final double[] groundPickup = {1, 50};          
    public static final double[] feederStation = {6, 101};   //originally 3      
  
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;
      
  // Joystick Deadband
    public static final double DEADBAND        = 0.03;
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

    public static final int startingConfiguration = 18;
    public static final int feederStation = 10;
    public static final int groundPickup = 11;
  

    public static final int IntakeIn = 13;
    public static final int IntakeOut = 12;

    public static final int ClimberLockSwitch = 14;
    public static final int ClimberSwitch = 15;

    public static final int ElevatorJoystick = 0;
    public static final int Wristjoystick = 1;

    public static final int ClimberJoystickX = 2;
    public static final int ClimberJoystickY = 3; 
  }



  public static final class GripperConstants {
    public static final String CANBUS_NAME = "canivore";
    public static final int WRIST_MOTOR_CANID = 19;
    public static final int GRIPPER_MOTOR1_CANID = 20;

    public static final int HOME_SENSOR_PORT = 4;
    public static final int ROT_SENSOR_PORT = 5;

    public static final double WRIST_ALLOWED_ERROR = 5; //TODO: Tune this value
    
    public static final double WRIST_P = 0.17; //.5
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0.02; //.13

    public static final double RAMP_UP = .1;

    public static final int CORAL_SENSOR_PORT = 2; 
    public static final int ALGAE_SENSOR_PORT = 3; 

    // 0 is Home (UP) and -86 is Down rotated clockwise as viewed from right side 
    public static final double CLOSE_POS = -20.0;
    public static final double MID_POS = -40.0;
    public static final double FAR_POS = -80.0;

    public static final int MIN_POS = 0;
    public static final int MAX_POS = 102;

   public static final double IntakeInSpeed = 0.8; // positive for in
   public static final double IntakeOutSpeed = -0.9; // negitive for out

   public static final double IntakeShootSpeed = -1.0; // Shoot the ball\

   public static final double WRIST_TOLERANCE = 2.0;
    
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MAIN_CANID = 21;
    public static final int CLIMBER_FOLLOWER_CANID = 22;
    public static final String Climber_BUS = "canivore";

    public static final int HOME_SENSOR_PORT = 6;
    public static final int DEPLOYED_SENSOR_PORT = 7;

    public static final int CLIMBER_RIGHT_LOCK_SERVO_PORT = 0;
    public static final int CLIMBER_LEFT_LOCK_SERVO_PORT = 1;

    public static final double CLIMBER_RIGHT_LOCKED_SETPOINT = 0.25;
    public static final double CLIMBER_LEFT_LOCKED_SETPOINT = 0.0;

    public static final double CLIMBER_RIGHT_UNLOCKED_SETPOINT = 0.0;
    public static final double CLIMBER_LEFT_UNLOCKED_SETPOINT = 0.25;

    public static final double CLIMBER_KP = .5;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.01;
    public static final double CLIMBER_MAX_OUTPUT = 0.5;
    public static final double CLIMBER_RAMPUP =  0.05;


    public static final double CLIMBER_DEPLOY_POSITION = 190.0; // Gets corrected to negative in code
    public static final double CLIMBER_RETRACT_POSITION = 60.0;
    public static final double CLIMBER_HOME_POSITION = 0.0;

    public static final double MAX_POS = -200.0;
    public static final double MIN_POS = 0;

    public static final double CLIMBER_DEADBAND = 0.2;

    public static final double CLIMBER_OVERRIDE_SPEED = .1;
    public static final double CLIMBER_RETRACT_SPEED = .9;
    public static final double CLIMBER_DEPLOY_SPEED = .6;

    public static final double CLIMBER_MAX_RETRACT_SPEED = 0.8;
  }

  public static final class LightingConstants { 
    public static final int CANDLE_CANID = 23; //TODO: Change this value
    public static final String CANDLE_BUS = "rio";
  }

  public static final class ControllerFunction {
    // Implements an expenential curve to the joystick input to allow for more precise control at lower speeds
    public static final double POWER = 2.7;
  }

  public static final class ShuffleboardConstatns {
    public static final String DriverDataTabName = "Driver Data";
    public static final String DebugDataTabName = "Debug Data";
  }

  public static final class LimelightConstants {
    public static final boolean DEBUG_ENABLED = true;

    public static final boolean FORCE_MIN_SPEED_AUTO = false; //Force a minimum speed during auto
    public static final double MIN_FORCED_SPEED_AUTO = 0.2; //Minimum speed during auto

    public static final boolean FORCE_MIN_SPEED_TELEOP = false; //Force a minimum speed during teleop
    public static final double MIN_FORCED_SPEED_TELEOP = 0.2; // Minimum speed during teleop

    public static final String ll_FRONT_RIGHT_NAME = "limelight-right";
    public static final double FWD_OFFSET = -0.0508; // Forward offset (meters)
    public static final double SIDE_OFFSET = 0.0762; // Side offset (meters)
    public static final double HEIGHT_OFFSET = 0.2921; // Height offset (Meters)
    public static final double ROLL = 0; // Roll Degrees
    public static final double PITCH = 0; // Pitch Degrees
    public static final double YAW = 15; // Yaw

    public static final String ll_FRONT_LEFT_NAME = "limelight-left";
    public static final double FWD_OFFSET_ALT = -0.0508; // Forward offset (meters)
    public static final double SIDE_OFFSET_ALT = -0.0762; // Side offset (meters)
    public static final double HEIGHT_OFFSET_ALT = 0.2921; // Height offset (Meters)
    public static final double ROLL_ALT = 0; // Roll Degrees
    public static final double PITCH_ALT = 0; // Pitch Degrees
    public static final double YAW_ALT = -15; // Yaw

    public static final double DONT_SEE_TAG_WAIT_TIME = 5.0;
    public static final double POSE_VALIDATION_TIME = 1.0;
    public static final double[] REEF_TOLERANCE_ALIGNMENT ={0.05,0.03,1}; // X (fwd-back), Y(left-right), Z(rot)
    public static final double[] REEF_CONST_SPEEDS ={.3,.3,0.2};

    // PID Constants, rotation has separate constants due to difference in units (meters vs degrees)
    public static final double REEF_X_KP = 3; // FWD/Back
    public static final double REEF_X_KI = 0.0; // FWD/Back
    public static final double REEF_X_KD = 0.17; // FWD/Back

    public static final double REEF_Y_KP = 4; // Left/Rgiht
    public static final double REEF_Y_KI = 0.0; // Left/Rgiht
    public static final double REEF_Y_KD = 0.15; // Left/Rgiht

    public static final double REEF_ROT_KP = 0.095;
    public static final double REEF_ROT_KI = 0.0;
    public static final double REEF_ROT_KD = 0.0;

    public static final int LL_SAMPLING = 6;

    // Offsets to match for placement
    // Using targetpose_botspace
    // Use LL interface to get values
    public static final double[] LEFT_CORAL_RCAM_OFFSETS = {
      .47, // TZ (meters) (Distance Away Z)
      .13, // TX (meters) (Distance Left Right) 
      4.5 // RY (meters)
    };

    public static final double[] RIGHT_CORAL_RCAM_OFFSETS = {
      .47, // TZ (meters) (Distance Away Z)
      -.15, // -.2 Last TX (meters) (Distance Left Right)
      2.8 // RY (meters)
    };

    public static final double[] RIGHT_CORAL_LCAM_OFFSETS = {
      .47, // TZ (meters) (Distance Away Z)
      -.13, // -.2 Last TX (meters) (Distance Left Right)
      -1.15 // RY (meters)
    };


    public static final double[] LEFT_CORAL_LCAM_OFFSETS = {
      .47, // TZ (meters) (Distance Away Z)
      .15, // TX (meters) (Distance Left Right) 
      -.8 // RY (meters)
    };

    public static final double[] CENTER_CORAL_OFFSETS = {
      .5, // TZ
      .05, // TX
      3.6 // RY
    };

  }
}
