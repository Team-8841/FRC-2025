package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimberToPosition extends Command {

    Climber m_Climber;
    double m_target;

    public MoveClimberToPosition(Climber climber, double target) {
        m_Climber = climber;
        m_target = target;

        this.addRequirements(m_Climber);
    }

    @Override
    public void execute() {
        m_Climber.setClimberPosition(m_target);
    }

    
    
}
