package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class ElevatorSubsystem extends SubsystemBase {

  private StatusSignal<Angle> position;

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  /* Keep a brake request so we can disable the motor */
  private final CoastOut m_coast = new CoastOut();

  private final TalonFX m_elevator_leader= new TalonFX(ElevatorConstants.M1_CANID, ElevatorConstants.CANBUS_NAME);
  private final TalonFX m_elevator_follower = new TalonFX(ElevatorConstants.M2_CANID, ElevatorConstants.CANBUS_NAME);


  // Elevator Range is from 0 to 98. With 98 being fully up and touching top
  // Grabber range is from 0 to -86. With negative rotating Clock Wise from home looking from side with front right
  private double setPoint = 5.0; 

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
            leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ElevatorConstants.RAMP_UP; // In seconds to ramp up to 100
            m_elevator_leader.getConfigurator().apply(leaderConfig);
        
        follower = new Follower(m_elevator_leader.getDeviceID(), true); // Inverted Follower
        m_elevator_follower.setControl(follower);

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
        if (targetposition > ElevatorConstants.MIN_POS && targetposition < ElevatorConstants.MAX_POS) {
            setPoint = targetposition;
            m_elevator_follower.setControl(follower);
            m_elevator_leader.setControl(new PositionDutyCycle(setPoint));
        }
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