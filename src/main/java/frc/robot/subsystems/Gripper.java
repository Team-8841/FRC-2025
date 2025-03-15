package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase{

    private final TalonFX m_gripper_motor = new TalonFX(GripperConstants.GRIPPER_MOTOR1_CANID, GripperConstants.CANBUS_NAME);
    private final TalonFX m_wrist_motor = new TalonFX(GripperConstants.WRIST_MOTOR_CANID, GripperConstants.CANBUS_NAME);

    private final PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);

    private DigitalInput coralSensor, algaeSensor, homeSensor, rotatedSensor;

    private double wristSetPoint;
    private final NeutralOut m_brake = new NeutralOut();

    private final DutyCycleOut StopMotor = new DutyCycleOut(0);

    private boolean getoffsensor = false;


    public Gripper() {
        TalonFXConfiguration wristConfig =  new TalonFXConfiguration();
        TalonFXConfiguration grippeConfig = new TalonFXConfiguration();

        m_PDH.setSwitchableChannel(false); // Start with the switchable channel off

        // Wrist Config
        wristConfig.Slot0.kP = GripperConstants.WRIST_P;
        wristConfig.Slot0.kI = GripperConstants.WRIST_I;
        wristConfig.Slot0.kD = GripperConstants.WRIST_D;

        wristConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = GripperConstants.RAMP_UP;
        m_wrist_motor.getConfigurator().apply(wristConfig);
        m_wrist_motor.setPosition(0);
        wristSetPoint = 0; // Start at neutral position

        grippeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_gripper_motor.getConfigurator().apply(grippeConfig);

        coralSensor = new DigitalInput(GripperConstants.CORAL_SENSOR_PORT);
        algaeSensor = new DigitalInput(GripperConstants.ALGAE_SENSOR_PORT);

        homeSensor = new DigitalInput(GripperConstants.HOME_SENSOR_PORT);
        rotatedSensor = new DigitalInput(GripperConstants.ROT_SENSOR_PORT);
    }

    public void setGripperSpeed(double speed) {
        m_gripper_motor.set(speed);
    }

    public void stopGripper() {
        m_gripper_motor.set(0);
        m_gripper_motor.setControl(m_brake);
    }

    public void enableSwitchablePDHChannel(boolean enabled) {
        m_PDH.setSwitchableChannel(enabled);
    }

    //TODO: Perform check for elevator configuration to avoid hitting elevator
    public void setWristPosition(double position) {
        /*if (position > 0) {
            position = position * -1; //Should always be negative
        }*/
        if (position >= GripperConstants.MIN_POS && position <= GripperConstants.MAX_POS) { // Direction is always NEGATIVE
            wristSetPoint = position;
            if (homeSensor.get() == false) // Home sensor is triggered 
            {
                getoffsensor = true; // Set variable to allow to leave home
            }
            if(!homeSensor.get() && position < m_wrist_motor.getPosition().getValueAsDouble()){
                stopGripper();
            }else if(!rotatedSensor.get() && position > m_wrist_motor.getPosition().getValueAsDouble()) {
                stopGripper();
            } else {
                m_wrist_motor.setControl(new PositionDutyCycle(wristSetPoint));
            }
        }
    }

    public double getWristPosition() {
        return m_wrist_motor.getPosition().getValueAsDouble();
    }

    public boolean wristAtPosition() {
        return Math.abs(getWristPosition() - wristSetPoint) <= GripperConstants.WRIST_ALLOWED_ERROR;
    }

    public boolean isCoralDetected() {
        return !coralSensor.get();
    }

    public boolean isAlgaeDetected() {
        return !algaeSensor.get();
    }
    
    @Override
    public void periodic() {
        if (homeSensor.get() == false || rotatedSensor.get() == false) {
            if (getoffsensor == false) {
                m_wrist_motor.setControl(StopMotor);
            }   
        }

        if (homeSensor.get() == true && getoffsensor == true)
        {
            getoffsensor = false;
        }

        // Put your periodic code here, called once per scheduler run
        SmartDashboard.putNumber("#[Wrist]: Setpoint", wristSetPoint);
        SmartDashboard.putBoolean("#[Gripper]: Coral", isCoralDetected());
        SmartDashboard.putBoolean("#[Gripper]: Algae", isAlgaeDetected());
        SmartDashboard.putNumber("#[Wrist]: Position", m_wrist_motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("#[Elevator]: Home Sensor", homeSensor.get());
        SmartDashboard.putBoolean("#[Wrist]: Rotated Sensor", rotatedSensor.get()); 
        SmartDashboard.putBoolean("#[Wrist]: At Position", wristAtPosition());   
    }
}
