package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class IntakeAndWait extends Command{

    private Gripper m_gripper;

    public IntakeAndWait(Gripper gripper) {
        this.m_gripper = gripper;

        addRequirements(gripper);
    }


    @Override
    public void execute() {

        if(!m_gripper.isCoralDetected()) {

            m_gripper.setGripperSpeed(GripperConstants.IntakeInSpeed);
            m_gripper.enableSwitchablePDHChannel(false);

        }
    }

    @Override
    public boolean isFinished() {
        if (m_gripper.isCoralDetected()) {
            m_gripper.setGripperSpeed(0);
            m_gripper.enableSwitchablePDHChannel(true);
            return true;
        } else {
            return false;
        }
    }
    
}
