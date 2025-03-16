package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class AutoMoveToFeederStation extends SequentialCommandGroup {


    public AutoMoveToFeederStation(ElevatorSubsystem m_elevator, Gripper m_Gripper, double[] targets, boolean isAlgae) {
        addCommands(

            new AutoMoveWristToSetpoint(m_elevator, m_Gripper, targets),
            new AutoMoveElevatorToSetpoint(m_elevator, m_Gripper, targets, isAlgae)
            
        



        );
    }
    
}
