package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DriveClimberWithJoystick extends Command {

    private Climber m_Climber;
    private double m_input;
    private boolean invertInput;
    
    public DriveClimberWithJoystick(double input, Climber climber, Boolean invert){
            this.m_Climber = climber;
            this.m_input =  input;
            this.invertInput = invert;

            this.addRequirements(climber);
    }

    @Override
    public void execute() {
      if (invertInput == true)
      {
        m_input = m_input * -1;
      }
      if (m_input > Constants.ClimberConstants.CLIMBER_DEADBAND){ // Extending
        if (m_Climber.getOutSensor()) { // False is triggered
          m_Climber.driveClimber(-.3);
        } else {
          m_Climber.driveClimber(0);
        }
      } else if (m_input < (-1 * Constants.ClimberConstants.CLIMBER_DEADBAND)) // Retract
      {
        if(m_Climber.getInSensor()) { // False is tirggered
          m_Climber.driveClimber(.5);
        } else {
          m_Climber.driveClimber(0);
        }
      } else {
        m_Climber.driveClimber(0);
      }
    }
    
}
