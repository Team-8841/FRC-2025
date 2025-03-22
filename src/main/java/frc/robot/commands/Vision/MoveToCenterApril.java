package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;


public class MoveToCenterApril extends Command {

    private Vision m_vision; 
    private SwerveSubsystem m_drive;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;
    private double TX_SETPOINT,TY_SETPOINT,ROT_SETPOINT;
    private double xSpeed, ySpeed, rotValue;
    private double TX, TY, ROT;
    private boolean firstLoop; //Ensures speeds are calculated at least once prior to finishing command

    public MoveToCenterApril(Vision m_vision, SwerveSubsystem m_drive) // 0:Left, 1:Right
    {
        this.m_vision = m_vision;
        this.m_drive = m_drive;
         // Target Center
            TX_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[2];
       
        
        this.addRequirements(this.m_drive);
    }


    @Override
    public void initialize()
    {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();
        this.firstLoop = false;

        tagID = LimelightHelpers.getFiducialID(m_vision.getLLName());
    }

    @Override
    public void execute()
    {
        double[][] sampled_positions = new double[LimelightConstants.LL_SAMPLING][6];
        double[] positions = new double[6];
        if (LimelightHelpers.getTV(m_vision.getLLName()) && LimelightHelpers.getFiducialID(m_vision.getLLName()) == tagID) {
            this.dontSeeTagTimer.reset();
      
            //Average current position 
            for (int i = 0; i < LimelightConstants.LL_SAMPLING; i++){
                sampled_positions[i] = LimelightHelpers.getTargetPose_RobotSpace(m_vision.getLLName());
            }
            for (int r = 0; r < 6; r++){   
                double avg = 0;
                for(int i = 0; i < LimelightConstants.LL_SAMPLING; i++)
                {
                    avg = avg + sampled_positions[i][r];
                }
                positions[r] = avg/LimelightConstants.LL_SAMPLING;
            }

            // Averaged TX, TY, ROT values
            TX  = positions[2];
            TY = positions[0];
            ROT = positions[4];

            
            xSpeed = getConstSpeed(TX,TX_SETPOINT,LimelightConstants.REEF_TOLERANCE_ALIGNMENT[0],LimelightConstants.REEF_CONST_SPEEDS[0]);
            ySpeed = -1*getConstSpeed(TY,TY_SETPOINT,LimelightConstants.REEF_TOLERANCE_ALIGNMENT[1],LimelightConstants.REEF_CONST_SPEEDS[1]);
            rotValue = -1*getConstSpeed(ROT,ROT_SETPOINT,LimelightConstants.REEF_TOLERANCE_ALIGNMENT[2],LimelightConstants.REEF_CONST_SPEEDS[2]);

            //SmartDashboard.putNumber("$[VISION]_XSPEED", xSpeed);
            //SmartDashboard.putNumber("$[VISION]_YSPEED", ySpeed);
            //SmartDashboard.putNumber("$[VISION]_ROTSPEED", rotValue);
            
            //System.out.println("TX: " + TX + ", TY:" + TY + ", ROT:" + ROT);
            //System.out.println("TX Set:" + TX_SETPOINT + ", TY Set:" +TY_SETPOINT + ", Rot Set:" +ROT_SETPOINT);
            //System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed +", RotSpeed: " + rotValue);
            //System.out.println(); 

            firstLoop = true;
            
            m_drive.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

          } else {
            m_drive.drive(new Translation2d(0,0), 0, false);
          }
      
         // SmartDashboard.putNumber("$[VISION]_POSEValidTimer", stopTimer.get());
    }
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted)
   {
        m_drive.drive(new Translation2d(0,0), 0, false);
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished()
   {    
    if (xSpeed == 0 && ySpeed == 0 && rotValue == 0 && firstLoop)
    {
        return true;
    }
    return this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME);
   }



   public double getConstSpeed(double current_pos, double set_position, double tolerance, double speed)
   {
    double difference = current_pos - set_position;
    if (Math.abs(difference) > tolerance)
    {
        if (difference > 0)
        {
            return speed;
        }else{
            return -1*speed;
        }
    }
    return 0;
   }
  
}

