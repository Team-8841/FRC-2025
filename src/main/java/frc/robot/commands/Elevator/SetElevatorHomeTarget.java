package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHomeTarget extends Command {

    private ElevatorSubsystem m_elevator;
    private double[] m_targets;

    public SetElevatorHomeTarget(ElevatorSubsystem subsystem, double[] target) {
        this.m_elevator = subsystem;
        this.m_targets = target;

        this.addRequirements(subsystem);
    }


    @Override
    public void execute() {
        m_elevator.setElevatorHomeTarget(m_targets);
    }
    
}
