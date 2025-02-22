package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

public class DriveClimberWithJoystick extends Command {

    private Climber m_Climber;
    private CommandJoystick joystick;
    private boolean invertInput;
    private double m_input;
    private boolean CLIMBER_OVERRIDE;
    
    public DriveClimberWithJoystick(CommandJoystick input, Climber climber, Boolean invert){
            this.m_Climber = climber;
            this.joystick =  input;
            this.invertInput = invert;

            this.addRequirements(climber);
    }

    @Override
    public void execute() {
      CLIMBER_OVERRIDE = joystick.button(Constants.OperatorConstants.ElevatorLock).getAsBoolean();
      m_input = joystick.getRawAxis(OperatorConstants.WristjoystickY);
      if (invertInput == true)
      {
        m_input = m_input * -1;
      }
      // Extend 
      if (m_input > Constants.ClimberConstants.CLIMBER_DEADBAND && !m_Climber.Extended){
        if (m_Climber.getOutSensor()) { // False is triggered
          m_Climber.driveClimber(-1 * Constants.ClimberConstants.CLIMBER_DEPLOY_SPEED);
        } else {
          m_Climber.Extended = true;
          m_Climber.driveClimber(0);
        }
      // Retract after extention
      } else if (m_input < (-1 * Constants.ClimberConstants.CLIMBER_DEADBAND) && m_Climber.Extended){
        if(m_Climber.getInSensor()) { // False is tirggered
          m_Climber.driveClimber(Constants.ClimberConstants.CLIMBER_RETRACT_SPEED);
        } else {
          m_Climber.driveClimber(0);
        }
      } else if (CLIMBER_OVERRIDE) { // Override 
        if (m_input > Constants.ClimberConstants.CLIMBER_DEADBAND) {
          m_Climber.driveClimber(-1 * Constants.ClimberConstants.CLIMBER_OVERRIDE_SPEED); // Negative
        } else if(m_input < (-1 * Constants.ClimberConstants.CLIMBER_DEADBAND)){
          m_Climber.driveClimber(Constants.ClimberConstants.CLIMBER_OVERRIDE_SPEED);
        } else {
          m_Climber.driveClimber(0);
        }
      } else {
        m_Climber.driveClimber(0);
      }
  }

  @Override
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return false;
  }
}
