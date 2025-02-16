package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class DriveClimberWithJoystick extends Command {

    private Climber m_Climber;
    private double m_input;
    private boolean m_inSensor, m_outSensor;
    
    public DriveClimberWithJoystick(double input,Boolean outSensor, Boolean inSensor, Climber climber){
            this.m_Climber = climber;
            this.m_input =  input;
            this.m_inSensor = inSensor;
            this.m_outSensor = outSensor;

            this.addRequirements(climber);
    }

    @Override
    public void execute() {
        if(m_input > 0.3) {
            m_Climber.driveClimber(-0.3);
          }else if(m_input < -0.3) {
            m_Climber.driveClimber(0.3);
          } else {
            m_Climber.driveClimber(0);
          }
    }
    
}
