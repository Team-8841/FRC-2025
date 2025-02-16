package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class IntakeAndWait extends Command{

    private Gripper m_gripper;
    private boolean m_coralSensor;

    public IntakeAndWait(boolean coralSensor, Gripper gripper) {
        this.m_gripper = gripper;
        this.m_coralSensor = coralSensor;

        addRequirements(gripper);
    }


    @Override
    public void execute() {

        if(!m_coralSensor){
            m_gripper.setGripperSpeed(GripperConstants.IntakeInSpeed);
        } else {
            m_gripper.setGripperSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_coralSensor == true;
    }
    
}
