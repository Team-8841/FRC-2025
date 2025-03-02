package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;

public class MoveElevatorToSetpoint extends Command{   
    private ElevatorSubsystem m_Elevator;
    private Gripper m_gripper;

    public MoveElevatorToSetpoint(ElevatorSubsystem elevator) {
        this.m_Elevator = elevator;

        this.addRequirements(m_Elevator);
    }

    @Override
    public void execute() {
        if(m_Elevator.getElevatorReadyStatus()) {
            m_Elevator.setElevatorPosition(m_Elevator.getElevatorTarget()[0]);
        }
    }

    @Override
    public boolean isFinished() {
        return m_Elevator.elevatorAtSetpoint();
    }
}
