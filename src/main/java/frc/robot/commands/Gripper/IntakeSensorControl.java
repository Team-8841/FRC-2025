package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class IntakeSensorControl extends Command{

    private final Gripper m_gripper;
    private ElevatorSubsystem m_elevator;
    private boolean m_in, m_out;

    public IntakeSensorControl(boolean in, boolean out, Gripper gripper, ElevatorSubsystem elevator){
        this.m_gripper = gripper;
        this.m_in = in;
        this.m_out = out;
        this.m_elevator = elevator;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if(m_in && !m_gripper.isCoralDetected() && !m_gripper.isAlgaeDetected()){ // No game piece is detected

            m_gripper.setGripperSpeed(GripperConstants.IntakeInSpeed);
            m_gripper.enableSwitchablePDHChannel(false); // Light off

        }else if(m_out){ // Switch out

            m_gripper.setGripperSpeed(GripperConstants.IntakeOutSpeed);

            if(m_gripper.isAlgaeDetected() || m_gripper.isCoralDetected()){ // Game piece is detected
                m_gripper.enableSwitchablePDHChannel(true); // Light on
            } else {
                m_gripper.enableSwitchablePDHChannel(false); // Light off
            }

        }else if(m_elevator.getAlgaeState() && m_gripper.isAlgaeDetected()) { // Has game piece and is algae

            m_gripper.setGripperSpeed(0);
            m_gripper.enableSwitchablePDHChannel(true); // Light on

        }else if(!m_elevator.getAlgaeState() && m_gripper.isCoralDetected()){ // Has game piece and is coral

            m_gripper.setGripperSpeed(0);
            m_gripper.enableSwitchablePDHChannel(true); // Light on

        }else if(!m_in && !m_out) { // Switch if off

            m_gripper.setGripperSpeed(0);

            if(m_gripper.isAlgaeDetected() || m_gripper.isCoralDetected()){ // Game piece is detected
                m_gripper.enableSwitchablePDHChannel(true); // Light on
            } else {
                m_gripper.enableSwitchablePDHChannel(false); // Light off
            }

        }
    }
}
