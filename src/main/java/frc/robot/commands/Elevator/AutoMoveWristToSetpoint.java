package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class AutoMoveWristToSetpoint extends Command{

    private ElevatorSubsystem m_Elevator;
    private Gripper m_gripper;
    private double[] m_targets;

    public AutoMoveWristToSetpoint(ElevatorSubsystem elevator, Gripper gripper, double[] targets) {
        this.m_Elevator = elevator;
        this.m_gripper = gripper;
        this.m_targets = targets;

        this.addRequirements(m_Elevator, m_gripper);
    }

    @Override
    public void execute() {
        m_gripper.setWristPosition(m_targets[1]);
    }

    
    @Override
    public boolean isFinished() {
        return m_gripper.wristAtPosition();
    }
}
