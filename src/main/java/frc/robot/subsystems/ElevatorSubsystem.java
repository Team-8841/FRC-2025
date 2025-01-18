package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final ElevatorFeedforward feedforward;
    private final PIDController pidController;

    private final SparkMaxConfig config_m1;
    private final SparkMaxConfig config_m2;

    private double currentPosition;
    private double pidOutput;
    private double feedforwardOutput;
    private double setpoint;
    private double motorOutput; // Defines motor drivepoint

    private double min_setpoint; //Starting location

    public ElevatorSubsystem() {
        // Initialize Spark Max motor controllers
        motor1 = new SparkMax(ElevatorConstants.M1_CANID, MotorType.kBrushless); 
        motor2 = new SparkMax(ElevatorConstants.M2_CANID, MotorType.kBrushless); 


        // Configure Motor 1
        config_m1 = new SparkMaxConfig();
        config_m1.inverted(false).idleMode(IdleMode.kBrake);
        config_m1.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        config_m1.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        config_m1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor1.configure(config_m1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Motor 2 (Follows Motor 1, Inverted?)
        config_m2 = new SparkMaxConfig();
        config_m2.inverted(false).idleMode(IdleMode.kBrake);
        config_m2.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        config_m2.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        config_m2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config_m2.follow(motor1, true);
        motor2.configure(config_m2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Control Structures
        pidController = new PIDController(ElevatorConstants.PID_kS, ElevatorConstants.PID_kG, ElevatorConstants.PID_kA);
        feedforward = new ElevatorFeedforward(ElevatorConstants.FF_kS, ElevatorConstants.FF_kG, ElevatorConstants.FF_kV, ElevatorConstants.FF_kA);

        // Init Location
        min_setpoint = motor1.getEncoder().getPosition();
    }

    /**
     * Sets the elevator position using feedforward control.
     * @param velocity The desired velocity of the elevator in meters per second.
     * @param acceleration The desired acceleration of the elevator in meters per second squared.
     */
    public void setElevator(double new_setPoint) {
        // Ensure reasonable values
        if (new_setPoint < min_setpoint)
        {
            setpoint = min_setpoint;
        }
        else
        {
            setpoint = new_setPoint;
        }
    }

    /**
     * Stops the elevator.
     */
    public void stopElevator() {
        motor1.set(0);
    }

    public void releaseElevator() {
        config_m1.idleMode(IdleMode.kCoast);
        config_m2.idleMode(IdleMode.kCoast);

        motor1.configure(config_m1, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(config_m2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {

        // For Testing
        // Display debugging information on the SmartDashboard
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator Current Position", currentPosition);
        SmartDashboard.putNumber("Elevator PID Output", pidOutput);
        SmartDashboard.putNumber("Elevator Feedforward Output", feedforwardOutput);
        SmartDashboard.putNumber("Elevator Total Output", motorOutput);

        currentPosition = motor1.getEncoder().getPosition();
        pidOutput = pidController.calculate(currentPosition, setpoint);
        feedforwardOutput = feedforward.calculate(setpoint, 0); // Velocity = 0 for static target
        motorOutput = pidOutput + feedforwardOutput;
        motor1.set(motorOutput);

        // This method will be called once per scheduler run
        // Add any telemetry or logging here
    }
}