package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class IntakeSensorControl extends Command{

    private final Gripper m_gripper;
    private boolean m_in, m_out;

    public IntakeSensorControl(boolean in, boolean out, Gripper gripper){
        this.m_gripper = gripper;
        this.m_in = in;
        this.m_out = out;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if(m_in && !m_gripper.isCoralDetected() && !m_gripper.isAlgaeDetected()){
            m_gripper.setGripperSpeed(GripperConstants.IntakeInSpeed);
        }else if(m_out){
            m_gripper.setGripperSpeed(GripperConstants.IntakeOutSpeed);
        }else if(m_gripper.isCoralDetected() || m_gripper.isAlgaeDetected() || (!m_in && !m_out)){
            m_gripper.setGripperSpeed(0);
        }
    }
}
