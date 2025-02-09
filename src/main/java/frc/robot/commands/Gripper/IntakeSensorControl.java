package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class IntakeSensorControl extends Command{

    private final Gripper m_gripper;
    private final double m_speed;
    
    public IntakeSensorControl(Gripper gripper, double speed){
        this.m_gripper = gripper;
        this.m_speed = speed;
        addRequirements(this.m_gripper);
    }

    @Override
    public void execute() {
        m_gripper.setGripperSpeed(m_speed);
    }
    
}
