package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase{

    private SparkMax m_gripperMotor1, m_gripperMotor2, m_wristMotor;
    private SparkBaseConfig gripperConfig, wristConfig;
    private DigitalInput coralSensor, algaeSensor;

    private double wristSetPoint;


    public Gripper() {
        m_gripperMotor1 = new SparkMax(GripperConstants.GRIPPER_MOTOR1_CANID, MotorType.kBrushless);
        m_gripperMotor2 = new SparkMax(GripperConstants.GRIPPER_MOTOR2_CANID, MotorType.kBrushless);
        m_wristMotor = new SparkMax(GripperConstants.WRIST_MOTOR_CANID, MotorType.kBrushless);

        wristSetPoint = 0; // Start at neutral position

        coralSensor = new DigitalInput(GripperConstants.CORAL_SENSOR_PORT);
        algaeSensor = new DigitalInput(GripperConstants.ALGAE_SENSOR_PORT);

        gripperConfig = new SparkMaxConfig();
        wristConfig = new SparkMaxConfig();

        configureSparkMax(m_gripperMotor1, gripperConfig, false, IdleMode.kBrake, 20);
        configureSparkMax(m_gripperMotor1, gripperConfig, true, IdleMode.kBrake, 20);

        configureSparkPID(m_wristMotor, wristConfig, false, IdleMode.kBrake, 1000, 1000, 
            FeedbackSensor.kPrimaryEncoder, GripperConstants.WRIST_P, GripperConstants.WRIST_I, GripperConstants.WRIST_D);
    }

    private void configureSparkMax(SparkMax spark, SparkBaseConfig config, boolean inverted, IdleMode idleMode, int currentLimit) {
        config.inverted(inverted);
        config.idleMode(idleMode);
        config.smartCurrentLimit(currentLimit);


        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureSparkPID(SparkMax spark, SparkBaseConfig config, boolean inverted, IdleMode idleMode, double positionConversionFactor, double velocityConversionFactor, FeedbackSensor feedbackSensor, double p, double i, double d) {
        
        config
            .inverted(true)
            .idleMode(idleMode);
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
        m_wristMotor.set(position);
    }

    public boolean wristAtPosition() {
        double currentPosition = m_wristMotor.getEncoder().getPosition();
        return Math.abs(currentPosition - wristSetPoint) <= GripperConstants.WRIST_ALLOWED_ERROR;
    }

    public boolean isCoralDetected() {
        return coralSensor.get();
    }

    public boolean isAlgaeDetected() {
        return algaeSensor.get();
    }
    
}
