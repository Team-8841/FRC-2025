package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class LockClimber extends Command {

    private Climber m_climber;
    
    public LockClimber(Climber climber) {
        this.m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        m_climber.lockClimber();
    }

    @Override
    public boolean isFinished() {
        return m_climber.getRightLockPosition() == ClimberConstants.CLIMBER_RIGHT_LOCKED_SETPOINT && m_climber.getLeftLockPosition() == ClimberConstants.CLIMBER_LEFT_LOCKED_SETPOINT;
    }
    
}
