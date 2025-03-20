package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberLockPosition  extends Command{
    private Climber m_climber;

    private double m_targetPosition;

    public SetClimberLockPosition(Climber climber, double target) {
        this.m_climber = climber;
        this.m_targetPosition = target;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        m_climber.setClimberLocks(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return m_climber.getRightLockPosition() == m_targetPosition && m_climber.getLeftLockPosition() == m_targetPosition;
    }
    
}
