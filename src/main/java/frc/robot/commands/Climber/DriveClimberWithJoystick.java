package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerFunction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

public class DriveClimberWithJoystick extends Command {

    private Climber m_Climber;
    private CommandJoystick joystick;
    private boolean invertInput;
    private double m_input;
    
    public DriveClimberWithJoystick(CommandJoystick input, Climber climber, Boolean invert){
            this.m_Climber = climber;
            this.joystick =  input;
            this.invertInput = invert;

            this.addRequirements(climber);
    }

  public double convertJoystickQuadratic(double input)
  {
    if (input < 0) {
      return -((Math.pow(ControllerFunction.POWER, -input) - 1) / (ControllerFunction.POWER - 1));
    } else {
      return (Math.pow(ControllerFunction.POWER, input) - 1) / (ControllerFunction.POWER -1);
    }
  }
    

    @Override
    public void execute() {

      if(invertInput) {
        m_input = joystick.getRawAxis(OperatorConstants.ClimberJoystickY) * -1;
      } else {
        m_input = joystick.getRawAxis(OperatorConstants.ClimberJoystickY);
      }
      
      if(m_input > ClimberConstants.CLIMBER_DEADBAND) {
        if(m_Climber.getOutSensor()) {
          m_Climber.driveClimber(-ClimberConstants.CLIMBER_DEPLOY_SPEED);
        } else {
          m_Climber.driveClimber(0);
        }
      }else if(m_input < -ClimberConstants.CLIMBER_DEADBAND) {
        if(m_Climber.getInSensor()){
          m_Climber.driveClimber(convertJoystickQuadratic(-m_input));
        } else {
          m_Climber.driveClimber(0);
        } 
      }
      else 
      {
        m_Climber.driveClimber(0);
      }
    }
}
