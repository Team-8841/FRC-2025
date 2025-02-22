package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class ShootAlgae extends Command{

    private Gripper m_gripper;
    private double m_speed;

    public ShootAlgae(Gripper gripper, double speed)
    {
        this.m_gripper = gripper;
        this.m_speed = speed;

        this.addRequirements(gripper);
    }


    @Override
    public void execute() {
        m_gripper.setGripperSpeed(m_speed);
    }
    
}
