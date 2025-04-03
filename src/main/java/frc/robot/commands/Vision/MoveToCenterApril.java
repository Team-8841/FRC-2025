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
    private boolean xAtTolerance, yAtTolerance, rotAtTolerance;

    public MoveToCenterApril(Vision m_vision, SwerveSubsystem m_drive) // 0:Left, 1:Right
    {
        this.m_vision = m_vision;
        this.m_drive = m_drive;
        
        this.addRequirements(this.m_drive);
    }


    @Override
    public void initialize()
    {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        tagID = LimelightHelpers.getFiducialID(m_vision.getPrimaryCam());
    }

    @Override
    public void execute()
    {


        //if(LimelightHelpers.getTV(this.m_vision.getCamName(true))) {
            this.m_vision.setRightLLAsPrimary(true);
            // Target Center
            TX_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[2];
        //} 
        /*else if(LimelightHelpers.getTV(this.m_vision.getCamName(false))){
            this.m_vision.setRightLLAsPrimary(false);
            // Target Center
            TX_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.CENTER_CORAL_OFFSETS[2];
                
        }*/

        double[][] sampled_positions = new double[LimelightConstants.LL_SAMPLING][6];
        double[] positions = new double[6];
        this.m_vision.setRightLLAsPrimary(true);
        
        if (LimelightHelpers.getTV(m_vision.getPrimaryCam()) && LimelightHelpers.getFiducialID(m_vision.getPrimaryCam()) == tagID) {
            this.dontSeeTagTimer.reset();
      
            //Average current position 
            for (int i = 0; i < LimelightConstants.LL_SAMPLING; i++){
                sampled_positions[i] = LimelightHelpers.getTargetPose_RobotSpace(m_vision.getPrimaryCam());
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

            xSpeed = m_vision.xController.getOutput(TX, TX_SETPOINT);
            ySpeed = m_vision.yController.getOutput(TY, TY_SETPOINT);
            rotValue = m_vision.rotController.getOutput(ROT, ROT_SETPOINT);
            
            m_drive.drive(new Translation2d(-xSpeed, ySpeed), rotValue, false);

          } else {
            m_vision.setRightLLAsPrimary(true);
            m_drive.drive(new Translation2d(0,0), 0, false);
          }
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
    xAtTolerance = atTolerance(TX, TX_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[0]);
    yAtTolerance = atTolerance(TY, TY_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[1]);
    rotAtTolerance = atTolerance(ROT, ROT_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[2]);

    if (xAtTolerance && yAtTolerance && rotAtTolerance)
    {   m_vision.xController.reset();
        m_vision.yController.reset();
        m_vision.rotController.reset();
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

   public boolean atTolerance(double current_pos, double set_position, double tolerance)
   {
    double difference = current_pos - set_position;
    if (Math.abs(difference) > tolerance)
    {
        return false;
    }
    return true;
   }

}


