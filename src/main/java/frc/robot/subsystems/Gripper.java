package frc.robot.subsystems;

<<<<<<< Updated upstream
=======

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase{
<<<<<<< Updated upstream
=======
    private TalonFX m_gripperMotor1,m_wristMotor,m_gripperMotor2;
    private TalonFX gripperConfig, wristConfig;
    private DigitalInput coralSensor, algaeSensor;

    private double wristSetPoint;

    public Gripper() {
        m_gripperMotor2 = new TalonFX(GripperConstants.GRIPPER_MOTOR2_CANID);
        m_wristMotor = new TalonFX(GripperConstants.WRIST_MOTOR_CANID);

        wristSetPoint = 0; // Start at neutral position

        coralSensor = new DigitalInput(GripperConstants.CORAL_SENSOR_PORT);
        algaeSensor = new DigitalInput(GripperConstants.ALGAE_SENSOR_PORT);

        configureTalonFX(m_gripperMotor1);
        configureTalonFX(m_gripperMotor2);
        

        configureTalonFX(m_gripperMotor1, gripperConfig, false, NeutralMode.Brake, 20);
        configureTalonFX(m_gripperMotor1, gripperConfig, true, NeutralMode.Brake, 20);

        configureTalonFXPID(m_wristMotor, wristConfig, false, NeutralMode.Brake, 1000, 1000, 
            FeedbackSensor.kPrimaryEncoder, GripperConstants.WRIST_P, GripperConstants.WRIST_I, GripperConstants.WRIST_D);
    }
// i wasn't able to do this part yet because i couldn't figure out how to invert
    private void configureTalonFX(TalonFX spark, TalonFXConfiguration config, boolean inverted, NeutralMode neutralMode, int currentLimit) {
        config.inverted(inverted);
        config.setNeutralMode(neutralMode);
        config.smartCurrentLimit(currentLimit);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        m_gripperMotor1.set(ControlMode.PercentOutput, speed);
        m_gripperMotor2.set(ControlMode.PercentOutput, speed);
    }

    public void setWristPosition(double position) {
        wristSetPoint = position;
        m_wristMotor.set(ControlMode.Position,position);
    }

    public boolean wristAtPosition() {
        double currentPosition = m_wristMotor.getSelectedSensorPosition();
        return Math.abs(currentPosition - wristSetPoint) <= GripperConstants.WRIST_ALLOWED_ERROR;
    }

    public boolean isCoralDetected() {
        return coralSensor.get();
    }

    public boolean isAlgaeDetected() {
        return algaeSensor.get();
    }
>>>>>>> Stashed changes
    
}



