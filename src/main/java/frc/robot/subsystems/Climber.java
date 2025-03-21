package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase{

    private TalonFX m_climberMain = new TalonFX(ClimberConstants.CLIMBER_MAIN_CANID, ClimberConstants.Climber_BUS);
    private TalonFX m_climberFollower = new TalonFX(ClimberConstants.CLIMBER_FOLLOWER_CANID, ClimberConstants.Climber_BUS);


    private ControlRequest follower;

    private Boolean m_hasExtended; 

    private DutyCycleOut stopMotor = new DutyCycleOut(0);

    private double setPoint = 0;

    private static DigitalInput stopSensor = new DigitalInput(ClimberConstants.DEPLOYED_SENSOR_PORT);
    private static DigitalInput homeSensor = new DigitalInput(ClimberConstants.HOME_SENSOR_PORT);

    private Servo m_lockServoRight = new Servo(ClimberConstants.CLIMBER_RIGHT_LOCK_SERVO_PORT);
    private Servo m_lockServoLeft = new Servo(ClimberConstants.CLIMBER_LEFT_LOCK_SERVO_PORT);

    private boolean CLIMBER_OVERRIDE;


    public Climber() {
        TalonFXConfiguration m_leaderConfig = new TalonFXConfiguration();
        m_leaderConfig.Slot0.kP = ClimberConstants.CLIMBER_KP;
        m_leaderConfig.Slot0.kI = ClimberConstants.CLIMBER_KI;
        m_leaderConfig.Slot0.kD = ClimberConstants.CLIMBER_KD;
        m_leaderConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 ratio for simplicity
        m_leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ClimberConstants.CLIMBER_RAMPUP; // In seconds to ramp up to 100

        TalonFXConfiguration m_followerConfig = new TalonFXConfiguration();
        m_followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        m_climberMain.getConfigurator().apply(m_leaderConfig);
        m_climberFollower.getConfigurator().apply(m_followerConfig);

        follower = new Follower(m_climberMain.getDeviceID(), true); // Inverted Follower
        m_climberFollower.setControl(follower);
        m_climberMain.setPosition(0);

        m_lockServoRight.setBoundsMicroseconds(2500, 0, 0, 0, 500);
        m_lockServoLeft.setBoundsMicroseconds(2500, 0, 0, 0, 500);

        m_lockServoRight.set(ClimberConstants.CLIMBER_RIGHT_LOCKED_SETPOINT);
        m_lockServoLeft.set(ClimberConstants.CLIMBER_LEFT_LOCKED_SETPOINT);

        m_hasExtended = false;

        CLIMBER_OVERRIDE = false;
    }

    public void stopClimber() {
        m_climberMain.set(0);
        m_climberFollower.set(0);
        //m_climberMain.setControl(stopMotor);
        //m_climberFollower.setControl(stopMotor);
    }

    public void setClimberPosition(double targetposition)
    {
        //TODO: Validate Elevator position 
        if (targetposition > 0)
        {
            targetposition = targetposition * -1;
        }
        if (targetposition > ClimberConstants.MAX_POS && targetposition <= ClimberConstants.MIN_POS)
            setPoint = targetposition;
            m_climberMain.setControl(new PositionVoltage(setPoint));
            m_climberFollower.setControl(follower);  
    }

    public void driveClimber(double speed){
        m_climberMain.setControl(new DutyCycleOut(speed));
        m_climberFollower.setControl(follower); 
    }


    public boolean getInSensor() {
        return homeSensor.get();
    }

    public boolean getOutSensor() {
        return stopSensor.get();
    }

    public void setClimberExtended(boolean state) {
        m_hasExtended = state;
    }

    public boolean hasClimberExtended() {
        return m_hasExtended;
    }

    //public boolean getClimberOveride() {
    //    return CLIMBER_OVERRIDE;
    //}

    public void setClimberLocks(double setpoint){
        m_lockServoRight.set(setpoint);
        m_lockServoLeft.set(setpoint);
    }

    public void lockClimber() {
        m_lockServoRight.set(ClimberConstants.CLIMBER_RIGHT_LOCKED_SETPOINT);
        m_lockServoLeft.set(ClimberConstants.CLIMBER_LEFT_LOCKED_SETPOINT);
    }

    public void unlockClimber() {
        m_lockServoRight.set(ClimberConstants.CLIMBER_RIGHT_UNLOCKED_SETPOINT);
        m_lockServoLeft.set(ClimberConstants.CLIMBER_LEFT_UNLOCKED_SETPOINT);
    }

    public double getRightLockPosition() {
        return m_lockServoRight.get();
    }

    public double getLeftLockPosition() {
        return m_lockServoLeft.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("#[Climber] Home Sensor", homeSensor.get());
        SmartDashboard.putBoolean("#[Climber] Extended Sensor", stopSensor.get());
        SmartDashboard.putBoolean("#[Climber] Has Extended", hasClimberExtended());
    }
    
}
