package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SetpointConstants;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class ElevatorSubsystem extends SubsystemBase {

  private StatusSignal<Angle> position;

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  /* Keep a brake request so we can disable the motor */
  private final CoastOut m_coast = new CoastOut();

  //private final CoastOut m_brake = new BrakeOut();

  private final TalonFX m_elevator_leader= new TalonFX(ElevatorConstants.M1_CANID, ElevatorConstants.CANBUS_NAME);
  private final TalonFX m_elevator_follower = new TalonFX(ElevatorConstants.M2_CANID, ElevatorConstants.CANBUS_NAME);


  // Elevator Range is from 0 to 98. With 98 being fully up and touching top
  // Grabber range is from 0 to -86. With negative rotating Clock Wise from home looking from side with front right
  private double setPoint = 5.0; 

  private double[] m_targetElevatorSetPoint;
  private double[] m_targetElevatorHomeSetPoint;
  private boolean m_readyToMove, m_isAlgaeState;

  private static DigitalInput topSensor = new DigitalInput(ElevatorConstants.DIO_TOPSENSOR);
  private static DigitalInput bottomSensor = new DigitalInput(ElevatorConstants.DIO_BOTTOMSENSOR); 

  private ControlRequest follower;


  private final DutyCycleOut StopMotor = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
            leaderConfig.Slot0.kP = ElevatorConstants.PID_P; // Proportional gain
            leaderConfig.Slot0.kI = ElevatorConstants.PID_I; // Integral gain
            leaderConfig.Slot0.kD = ElevatorConstants.PID_D; // Derivative gain
            leaderConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 ratio for simplicity
            //leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ElevatorConstants.RAMP_UP; // In seconds to ramp up to 100

            m_elevator_leader.getConfigurator().apply(leaderConfig);

            m_readyToMove = false;
            m_isAlgaeState = false;
            m_targetElevatorSetPoint = SetpointConstants.startingConfiguration;
            m_targetElevatorHomeSetPoint = SetpointConstants.feederStation;
        
        follower = new Follower(m_elevator_leader.getDeviceID(), true); // Inverted Follower
        m_elevator_follower.setControl(follower);

        /* Make sure we start at 0 */
        m_elevator_leader.setPosition(0);
    }

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
        if (targetposition >= ElevatorConstants.MIN_POS && targetposition <= ElevatorConstants.MAX_POS) {
            setPoint = targetposition;
            m_elevator_follower.setControl(follower);
            m_elevator_leader.setControl(new PositionDutyCycle(setPoint));
        }
    }

    public void driveElevator(double speed){
        //m_elevator_leader.setControl(Voltage);
    }

    public double getElevatorPosition() {
        return position.getValueAsDouble();
    }

    public boolean getTopSensor() {
        return !topSensor.get();
    }

    public boolean getBottomSensorO() {
        return !bottomSensor.get();
    }


    public void setElevatorTarget(double[] target)  {
        m_targetElevatorSetPoint[0] = target[0];
        m_targetElevatorSetPoint[1] = target[1];
    }

    public void setElevatorHomeTarget(double[] target){
        m_targetElevatorHomeSetPoint[0] = target[0];
        m_targetElevatorHomeSetPoint[1] = target[1];
    }

    public void setElevatorReadyStatus(boolean status) {
        m_readyToMove = status;
    }   

    public void setAlgaeState(boolean state) {
        m_isAlgaeState = state;
    }

    public boolean getAlgaeState() {
        return m_isAlgaeState;
    }

    public double[] getElevatorTarget() {
        return m_targetElevatorSetPoint;
    }

    public double[] getElevatorHomeTarget() {
        return m_targetElevatorHomeSetPoint;
    }  

    public boolean getElevatorReadyStatus() {
        return m_readyToMove;
    }

    public boolean elevatorAtSetpoint() {
        return Math.abs(getElevatorPosition() - setPoint) < ElevatorConstants.ELEVATOR_TOLERANCE;
    }

    @Override
    public void periodic() {

        // For Testing
        // Display debugging information on the SmartDashboard
        position = m_elevator_leader.getRotorPosition();
        SmartDashboard.putNumber("#[Elevator]: Position", position.getValueAsDouble());
        SmartDashboard.putNumber("#[Elevator]: SetPoint", setPoint);
        SmartDashboard.putBoolean("#[Elevator]: Top Sensor", getTopSensor());
        SmartDashboard.putBoolean("#[Elevator]: Bottom Sensor", getBottomSensorO());
        SmartDashboard.putBoolean("#[Elevator]: Ready" , getElevatorReadyStatus());
        SmartDashboard.putBoolean("#[Elevaotr]: At Setpoint", elevatorAtSetpoint());
        SmartDashboard.putNumber("#[Elevator Target]: ", getElevatorTarget()[0]);
        SmartDashboard.putNumber("#[Wrist Target]: ", getElevatorTarget()[1]);
        SmartDashboard.putNumber("#[Elevator Home Target]: ", getElevatorHomeTarget()[0]);
        SmartDashboard.putNumber("#[Wrist Home Target]: ", getElevatorHomeTarget()[1]);
        SmartDashboard.putBoolean("#[Elevator]: Algae State", m_isAlgaeState);
    }

}