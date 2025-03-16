package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveWristToSetpoint extends Command{   
    private ElevatorSubsystem m_Elevator;
    private Gripper m_gripper;

    public MoveWristToSetpoint(ElevatorSubsystem elevator, Gripper gripper) {
        this.m_Elevator = elevator;
        this.m_gripper = gripper;

        this.addRequirements(m_Elevator, m_gripper);
    }

    @Override
    public void execute() {
        m_gripper.setWristPosition(m_Elevator.getElevatorTarget()[1]);
    }

    @Override
    public boolean isFinished() {
        return m_gripper.wristAtPosition();
    }
}
