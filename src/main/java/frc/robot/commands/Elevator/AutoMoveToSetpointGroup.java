package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class AutoMoveToSetpointGroup extends SequentialCommandGroup {


    public AutoMoveToSetpointGroup(ElevatorSubsystem m_elevator, Gripper m_Gripper, double[] targets, boolean isAlgae) {
        addCommands(

            new AutoMoveElevatorToSetpoint(m_elevator, m_Gripper, targets, isAlgae),
            new AutoMoveWristToSetpoint(m_elevator, m_Gripper, targets)
        



        );
    }
    
}
