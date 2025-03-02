package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveToSetpointGroup extends SequentialCommandGroup {

    public MoveToSetpointGroup(ElevatorSubsystem m_elevator, Gripper m_Gripper) {
        addCommands(

         new MoveElevatorToSetpoint(m_elevator),

         new MoveWristToSetpoint(m_elevator, m_Gripper)



        );
    }
    
}
