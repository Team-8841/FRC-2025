package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase{

    private TalonFX m_climberMain = new TalonFX(ClimberConstants.CLIMBER_MAIN_CANID, ClimberConstants.Climber_BUS);
    private TalonFX m_climberFollower = new TalonFX(ClimberConstants.CLIMBER_FOLLOWER_CANID, ClimberConstants.Climber_BUS);

    private ControlRequest follower;

    private DutyCycleOut stopMotor = new DutyCycleOut(0);

    private double setPoint = 0;



    public void Climber() {
        TalonFXConfiguration m_leaderConfig = new TalonFXConfiguration();
        m_leaderConfig.Slot0.kP = ClimberConstants.CLIMBER_KP;
        m_leaderConfig.Slot0.kI = ClimberConstants.CLIMBER_KI;
        m_leaderConfig.Slot0.kD = ClimberConstants.CLIMBER_KD;
        m_leaderConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 ratio for simplicity
        m_leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ClimberConstants.CLIMBER_RAMPUP; // In seconds to ramp up to 100

        m_climberMain.getConfigurator().apply(m_leaderConfig);

        follower = new Follower(m_climberMain.getDeviceID(), true); // Inverted Follower
        m_climberFollower.setControl(follower);
    }

    public void stopClimber() {
        m_climberMain.setControl(stopMotor);
        m_climberFollower.setControl(stopMotor);
    }

        public void setClimberPosition(double targetposition)
    {
        if (targetposition > ElevatorConstants.MIN_POS && targetposition < ElevatorConstants.MAX_POS) {
            setPoint = targetposition;
            m_climberFollower.setControl(follower);
            m_climberFollower.setControl(new PositionDutyCycle(setPoint));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", m_climberMain.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber SetPoint", setPoint);
    }
    
}
