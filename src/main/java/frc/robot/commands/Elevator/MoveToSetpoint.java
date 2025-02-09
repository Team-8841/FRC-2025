package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveToSetpoint extends Command{   
    ElevatorSubsystem m_Elevator;
    Gripper m_gripper;
    double m_elevatorSetpoint, m_gripperSetpoint;

    public MoveToSetpoint(ElevatorSubsystem elevator, Gripper gripper, double[] targetPositions) {
        this.m_Elevator = elevator;
        this.m_gripper = gripper;

        this.m_elevatorSetpoint = targetPositions[0];
        this.m_gripperSetpoint = targetPositions[1];

        this.addRequirements(m_Elevator, m_gripper);
    }

    @Override
    public void execute() {
        m_Elevator.setElevatorPosition(m_elevatorSetpoint);
        m_gripper.setWristPosition(m_gripperSetpoint);
    }
    
}
