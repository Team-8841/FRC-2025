package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveToSetpoint extends Command{   
    private ElevatorSubsystem m_Elevator;
    private Gripper m_gripper;

    public MoveToSetpoint(ElevatorSubsystem elevator, Gripper gripper) {
        this.m_Elevator = elevator;
        this.m_gripper = gripper;

        this.addRequirements(m_Elevator, m_gripper);
    }

    @Override
    public void execute() {
        if(m_Elevator.getElevatorReadyStatus()) {
            m_Elevator.setElevatorPosition(m_Elevator.getElevatorTarget()[0]);
            m_gripper.setWristPosition(m_Elevator.getElevatorTarget()[1]);
        }
    }

    @Override
    public boolean isFinished() {
        return m_Elevator.elevatorAtSetpoint() && m_gripper.wristAtPosition();
    }
}
