package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorTarget extends Command {
    
    private ElevatorSubsystem m_elevator;
    private double[] m_targets;
    private boolean m_ready;

    public SetElevatorTarget(ElevatorSubsystem subsystem, double[] target, boolean ready) {
        this.m_elevator = subsystem;
        this.m_targets = target;
        this.m_ready = ready;

        this.addRequirements(subsystem);
    }


    @Override
    public void execute() {
        m_elevator.setElevatorTarget(m_targets);
        m_elevator.setElevatorReadyStatus(m_ready);
    }
    
}
