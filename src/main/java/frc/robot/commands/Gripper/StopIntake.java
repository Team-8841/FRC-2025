package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class StopIntake extends Command{

    private final Gripper m_gripper;
    private ElevatorSubsystem m_elevator;
    private boolean m_in, m_out;

    public StopIntake(boolean in, boolean out, Gripper gripper, ElevatorSubsystem elevator){
        this.m_gripper = gripper;
        this.m_in = in;
        this.m_out = out;
        this.m_elevator = elevator;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        
        m_gripper.setGripperSpeed(0);
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
