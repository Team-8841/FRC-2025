package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class ShootAlgae extends Command{

    private Gripper m_gripper;

    public ShootAlgae(Gripper gripper)
    {
        this.m_gripper = gripper;

        this.addRequirements(gripper);
    }


    @Override
    public void execute() {
        if(m_gripper.isAlgaeDetected()) {
            m_gripper.setGripperSpeed(GripperConstants.IntakeShootSpeed);
        }else{ 
            m_gripper.setGripperSpeed(0);
        }
    }
    
}
