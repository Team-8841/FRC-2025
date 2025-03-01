package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHomeTarget extends Command {

    private ElevatorSubsystem m_elevator;
    private double[] m_targets = new double[2];
    private boolean m_algaeState;

    public SetElevatorHomeTarget(ElevatorSubsystem subsystem, double[] target, boolean algaeState) {
        this.m_elevator = subsystem;
        this.m_targets[0] = target[0];
        this.m_targets[1] = target[1];
        this.m_algaeState = algaeState;

        this.addRequirements(subsystem);
    }


    @Override
    public void execute() {
        m_elevator.setElevatorHomeTarget(m_targets);
        m_elevator.setAlgaeState(m_algaeState);
    }
    
}
