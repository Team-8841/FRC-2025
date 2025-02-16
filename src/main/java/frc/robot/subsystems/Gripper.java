package frc.robot.subsystems;


import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase{

    private final TalonFX m_gripper_motor = new TalonFX(GripperConstants.GRIPPER_MOTOR1_CANID, GripperConstants.CANBUS_NAME);
    private final TalonFX m_wrist_motor = new TalonFX(GripperConstants.WRIST_MOTOR_CANID, GripperConstants.CANBUS_NAME);

    private DigitalInput coralSensor, algaeSensor, homeSensor, rotatedSensor;

    private double wristSetPoint;
    private final NeutralOut m_brake = new NeutralOut();

    private final DutyCycleOut StopMotor = new DutyCycleOut(0);

    private boolean getoffsensor = false;


    public Gripper() {
        TalonFXConfiguration wristConfig =  new TalonFXConfiguration();
        TalonFXConfiguration grippeConfig = new TalonFXConfiguration();

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

    //TODO: Perform check for elevator configuration to avoid hitting elevator
    public void setWristPosition(double position) {
        if (position > 0) {
            position = position * -1; //Should always be negative
        }
        if (position < GripperConstants.MIN_POS && position > GripperConstants.MAX_POS) { // Direction is always NEGATIVE
            wristSetPoint = position;
            if (homeSensor.get() == false) // Home sensor is triggered 
            {
                getoffsensor = true; // Set variable to allow to leave home
            }
            m_wrist_motor.setControl(new PositionDutyCycle(wristSetPoint));
        }
    }

    public boolean wristAtPosition() {
        double currentPosition = m_wrist_motor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - wristSetPoint) <= GripperConstants.WRIST_ALLOWED_ERROR;
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
        SmartDashboard.putNumber("Wrist Setpoint", wristSetPoint);
        SmartDashboard.putNumber("[Gripper]: Position", m_gripper_motor.getPosition().getValueAsDouble());
         SmartDashboard.putBoolean("[Gripper]: Coral", isCoralDetected());
        SmartDashboard.putBoolean("[Gripper]: Algae", isAlgaeDetected());
        SmartDashboard.putNumber("Wrist Position", m_wrist_motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Home Sensor", homeSensor.get());
        SmartDashboard.putBoolean("Rotated Sensor", rotatedSensor.get());    }

}
