package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase{

    private TalonFX m_gripperMotor1,m_wristMotor,m_gripperMotor2;
    private TalonFX gripperConfig, wristConfig;
    private DigitalInput coralSensor, algaeSensor;

    private double wristSetPoint;

    public Gripper() {
        m_gripperMotor1 =  new TalonFX(GripperConstants.GRIPPER_MOTOR1_CANID);
        m_gripperMotor2 = new TalonFX(GripperConstants.GRIPPER_MOTOR2_CANID);
        m_wristMotor = new TalonFX(GripperConstants.WRIST_MOTOR_CANID);

        wristSetPoint = 0; // Start at neutral position

        coralSensor = new DigitalInput(GripperConstants.CORAL_SENSOR_PORT);
        algaeSensor = new DigitalInput(GripperConstants.ALGAE_SENSOR_PORT);

        configureTalonFX(m_gripperMotor1);
        configureTalonFX(m_gripperMotor2);
        

        configureTalonFX(m_gripperMotor1, gripperConfig, false, NeutralMode.Brake, 20);
        configureTalonFX(m_gripperMotor1, gripperConfig, true, NeutralMode.Brake, 20);

        configureTalonFXPID(m_wristMotor, wristConfig, false, NeutralMode.Brake, 0, 0, 
            FeedbackSensor.kPrimaryEncoder, GripperConstants.WRIST_P, GripperConstants.WRIST_I, GripperConstants.WRIST_D);
    }
// i wasn't able to do this part yet because i couldn't figure out how to invert
private void configureTalonFX(TalonFX talon, TalonFXConfiguration config, Boolean inverted, NeutralModeValue neutralMode, int currentLimit) {
    config.MotorOutput.setInverted(inverted);
    config.MotorOutput.NeutralMode = neutralMode;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    talon.getConfigurator().apply(config);
}


//not sure bout this part i just tried to do what ever i could idk tho 
    private void configureTalonFXPID(TalonFX spark, TalonFXConfiguration config, boolean inverted, NeutralMode neutralMode, double positionConversionFactor, double velocityConversionFactor, FeedbackSensor feedbackSensor, double p, double i, double d) {
         
        config
            .inverted(true)
            .idleMode(neutralMode);
        config.encoder
            .positionConversionFactor(positionConversionFactor) //1000
            .velocityConversionFactor(velocityConversionFactor); //10000
        config.closedLoop
            .feedbackSensor(feedbackSensor) //FeedbackSensor.kPrimaryEncoder
            .pid(p, i, d);
    }

    @Override
    public void periodic() {
        // Put your periodic code here, called once per scheduler run
    }

    public void setGripperSpeed(double speed) {
        m_gripperMotor1.set(speed);
        m_gripperMotor2.set(speed);
    }

    public void setWristPosition(double position) {
        wristSetPoint = position;
        m_wristMotor.setControl(new PositionVoltage(position));
    }

    public boolean wristAtPosition() {
        double currentPosition = m_wristMotor.getPosition().getValue();
        return Math.abs(currentPosition - wristSetPoint) <= GripperConstants.WRIST_ALLOWED_ERROR;
    }

    public boolean isCoralDetected() {
        return coralSensor.get();
    }

    public boolean isAlgaeDetected() {
        return algaeSensor.get();
    }
    
}



