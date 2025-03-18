package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

        if(m_gripper.isAlgaeDetected() || m_gripper.isCoralDetected()){ // Game piece is detected
            m_gripper.enableSwitchablePDHChannel(true); // Light on
        } else {
            m_gripper.enableSwitchablePDHChannel(false); // Light off
        }
    }

    @Override
    public boolean isFinished() {
        return !m_gripper.isAlgaeDetected() && !m_gripper.isCoralDetected();
    }
    
}
