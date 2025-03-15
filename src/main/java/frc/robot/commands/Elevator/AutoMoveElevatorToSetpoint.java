package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class AutoMoveElevatorToSetpoint extends Command{

    private ElevatorSubsystem m_Elevator;
    private Gripper m_gripper;
    private double[] m_targets;
    private boolean m_isAlgae;

    public AutoMoveElevatorToSetpoint(ElevatorSubsystem elevator, Gripper gripper, double[] targets, boolean isAlgae) {
        this.m_Elevator = elevator;
        this.m_gripper = gripper;
        this.m_targets = targets;
        this.m_isAlgae = isAlgae;

        this.addRequirements(m_Elevator, m_gripper);
    }

    @Override
    public void execute() {

        m_Elevator.setAlgaeState(m_isAlgae);
        m_Elevator.setElevatorPosition(m_targets[0]);
    }

    
    @Override
    public boolean isFinished() {
        return m_Elevator.elevatorAtSetpoint();
    }
}
