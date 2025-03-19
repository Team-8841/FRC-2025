package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveToHome extends Command{

    private ElevatorSubsystem m_elevator;
    private Gripper m_gripper;
    private CommandJoystick m_copilot;


    public MoveToHome(ElevatorSubsystem elevator, Gripper gripper) {
        this.m_elevator = elevator;
        this.m_gripper = gripper;

        this.addRequirements(elevator, gripper);
    }

    @Override
    public void execute() {

        if(m_elevator.getElevatorHomeTarget()[0] == SetpointConstants.feederStation[0] && m_elevator.getElevatorHomeTarget()[1] == SetpointConstants.feederStation[1]){
            m_elevator.setAlgaeState(false);
        }

        m_elevator.setElevatorPosition(m_elevator.getElevatorHomeTarget()[0]);
        m_gripper.setWristPosition(m_elevator.getElevatorHomeTarget()[1]);
    }
    
}
