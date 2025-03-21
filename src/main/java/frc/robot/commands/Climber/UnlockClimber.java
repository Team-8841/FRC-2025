package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class UnlockClimber extends Command {

    private Climber m_climber;

    public UnlockClimber(Climber climber) {
        this.m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        m_climber.unlockClimber();
    }

    @Override
    public boolean isFinished() {
        return m_climber.getRightLockPosition() == ClimberConstants.CLIMBER_RIGHT_UNLOCKED_SETPOINT && m_climber.getLeftLockPosition() == ClimberConstants.CLIMBER_LEFT_UNLOCKED_SETPOINT;
    }
    
}
