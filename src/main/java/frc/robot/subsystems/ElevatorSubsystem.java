package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class ElevatorSubsystem extends SubsystemBase {
  private static final int MOTOR1_CAN_ID = 18; //Elevators, Leader
  private static final int MOTOR2_CAN_ID = 16;

  private StatusSignal<Angle> position;

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();
  private final CoastOut m_coast = new CoastOut();

  private final TalonFX m_elevator_leader= new TalonFX(MOTOR1_CAN_ID);
  private final TalonFX m_elevator_follower = new TalonFX(MOTOR2_CAN_ID);


  // Elevator Range is from 0 to -110. With -110 being fully extended up
  // Grabber range is from 0 to -86. With negative rotating Clock Wise from home looking from side with front right
  private double setPoint = -5.0;



  private int x = 0;

  private static DigitalInput topSensor;
  private static DigitalInput bottomSensor; 

  private ControlRequest follower;


  private final DutyCycleOut StopMotor = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
            leaderConfig.Slot0.kP = ElevatorConstants.PID_P; // Proportional gain
            leaderConfig.Slot0.kI = ElevatorConstants.PID_I; // Integral gain
            leaderConfig.Slot0.kD = ElevatorConstants.PID_D; // Derivative gain
            leaderConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 ratio for simplicity
            leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1; // In seconds to ramp up to 100
            m_elevator_leader.getConfigurator().apply(leaderConfig);
        
        follower = new Follower(m_elevator_leader.getDeviceID(), true); // Inverted Follower
        m_elevator_follower.setControl(follower);

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_elevator_leader.getConfigurator().apply(leaderConfig);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        /* Make sure we start at 0 */
        m_elevator_leader.setPosition(0);
    }

    /**
     * Sets the elevator position using feedforward control.
     * @param velocity The desired velocity of the elevator in meters per second.
     * @param acceleration The desired acceleration of the elevator in meters per second squared.
     */

    /**
     * Stops the elevator.
     */
    public void stopElevator() {
        m_elevator_leader.setControl(StopMotor);
        m_elevator_follower.setControl(StopMotor);
    }

    public void releaseElevator() {
      m_elevator_leader.setControl(m_coast);
      m_elevator_follower.setControl(m_coast);
    }

    public void resetEncoders()
    {
        m_elevator_leader.setPosition(0);
        m_elevator_follower.setPosition(0);
    }

    public void setElevatorPosition(double targetposition)
    {
        if (targetposition > 0) {
            targetposition = targetposition * -1; // Invert because should never be positive
        }
        setPoint = targetposition;
        m_elevator_follower.setControl(follower);
        m_elevator_leader.setControl(new PositionDutyCycle(setPoint));
    }

    @Override
    public void periodic() {

        // For Testing
        // Display debugging information on the SmartDashboard
        position = m_elevator_leader.getRotorPosition();
        m_elevator_leader.setControl(new PositionDutyCycle(setPoint));
        SmartDashboard.putNumber("Elevator Encoder", position.getValueAsDouble());
        SmartDashboard.putNumber("Elevator SetPoint", setPoint);
        if (topSensor.get() == false || bottomSensor.get() == false)
        {
            stopElevator();
        }
    }

}