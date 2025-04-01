package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Vision.MiniPID;
import java.util.Set;


public class MoveToApril extends Command {

    private Vision m_vision; 
    private SwerveSubsystem m_drive;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;
    private double TX_SETPOINT,TY_SETPOINT,ROT_SETPOINT;
    private double xSpeed, ySpeed, rotValue;
    private double TX, TY, ROT;
    private boolean xAtTolerance, yAtTolerance, rotAtTolerance, m_toRight, m_isAuto;

    public MoveToApril(Vision m_vision, SwerveSubsystem m_drive, boolean toRight, boolean isAuto)
    {
        this.m_vision = m_vision;
        this.m_drive = m_drive;
        this.m_toRight = toRight;
        this.m_isAuto = isAuto;
    
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
        double[][] sampled_positions = new double[LimelightConstants.LL_SAMPLING][6];
        double[] positions = new double[6];

        if(this.m_toRight && LimelightHelpers.getTV(this.m_vision.getCamName(true))) { // Moving to right and right has target
            this.m_vision.setRightLLAsPrimary(true); // Right is primary

            TX_SETPOINT = LimelightConstants.RIGHT_CORAL_RCAM_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.RIGHT_CORAL_RCAM_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.RIGHT_CORAL_RCAM_OFFSETS[2];

        } else if(this.m_toRight && !LimelightHelpers.getTV(this.m_vision.getCamName(true))){// Moving to right and right has no target
            this.m_vision.setRightLLAsPrimary(false); // Left is primary

            TX_SETPOINT = LimelightConstants.RIGHT_CORAL_LCAM_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.RIGHT_CORAL_LCAM_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.RIGHT_CORAL_LCAM_OFFSETS[2];

        } else if(!this.m_toRight && LimelightHelpers.getTV(this.m_vision.getCamName(false))) {// Moving to left and left has target
            this.m_vision.setRightLLAsPrimary(false); // Left is priamry

            TX_SETPOINT = LimelightConstants.LEFT_CORAL_LCAM_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.LEFT_CORAL_LCAM_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.LEFT_CORAL_LCAM_OFFSETS[2];

        } else if(!this.m_toRight && !LimelightHelpers.getTV(this.m_vision.getCamName(false))) { // Moving left and left has no target
            this.m_vision.setRightLLAsPrimary(true); // Right is primary

            TX_SETPOINT = LimelightConstants.LEFT_CORAL_RCAM_OFFSETS[0];
            TY_SETPOINT = LimelightConstants.LEFT_CORAL_RCAM_OFFSETS[1];
            ROT_SETPOINT = LimelightConstants.LEFT_CORAL_RCAM_OFFSETS[2];
        }


        if (LimelightHelpers.getTV(m_vision.getPrimaryCam()) && LimelightConstants.REEF_APRIL_TAGIDS.contains((int) LimelightHelpers.getFiducialID(m_vision.getPrimaryCam())) {
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

            
            //System.out.println("TX: " + TX + ", TY:" + TY + ", ROT:" + ROT);
            //System.out.println("TX Set:" + TX_SETPOINT + ", TY Set:" +TY_SETPOINT + ", Rot Set:" +ROT_SETPOINT);
            //System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed +", RotSpeed: " + rotValue);
            //System.out.println(); 

            xSpeed = m_vision.xController.getOutput(TX, TX_SETPOINT);
            ySpeed = m_vision.yController.getOutput(TY, TY_SETPOINT);
            rotValue = m_vision.rotController.getOutput(ROT, ROT_SETPOINT);
            

            if(this.m_isAuto) {

                if(LimelightConstants.FORCE_MIN_SPEED_AUTO) {

                    //If commanded speed(xSpeed) is less than MIN_FORCED_SPEED_AUTO, set it to MIN_FORCED_SPEED_AUTO or return xSpeed
                    xSpeed = (Math.abs(xSpeed) <= LimelightConstants.MIN_FORCED_SPEED_AUTO) ? LimelightConstants.MIN_FORCED_SPEED_AUTO : xSpeed;
                    ySpeed = (Math.abs(ySpeed) <= LimelightConstants.MIN_FORCED_SPEED_AUTO) ? LimelightConstants.MIN_FORCED_SPEED_AUTO : ySpeed;
                } else if (LimelightConstants.FORCE_CONST_SPEED_AUTO) {
                    xSpeed = getConstSpeed(TX, TX_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[0], LimelightConstants.REEF_CONST_SPEEDS[0]);
                    ySpeed = getConstSpeed(TY, TY_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[1], LimelightConstants.REEF_CONST_SPEEDS[0]);
                    rotValue = getConstSpeed(ROT, ROT_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[2], LimelightConstants.REEF_CONST_SPEEDS[0]);
                }

            } else if(LimelightConstants.FORCE_MIN_SPEED_TELEOP) {

                    xSpeed = (Math.abs(xSpeed) <= LimelightConstants.MIN_FORCED_SPEED_TELEOP) ? LimelightConstants.MIN_FORCED_SPEED_TELEOP : xSpeed;
                    ySpeed = (Math.abs(ySpeed) <= LimelightConstants.MIN_FORCED_SPEED_TELEOP) ? LimelightConstants.MIN_FORCED_SPEED_TELEOP : ySpeed;
            }
            
            if(LimelightConstants.DEBUG_ENABLED) {
                SmartDashboard.putNumber("$[VISION]_XSPEED", xSpeed);
                SmartDashboard.putNumber("$[VISION]_YSPEED", ySpeed);
                SmartDashboard.putNumber("$[VISION]_ROTSPEED", rotValue);
            }

            m_drive.drive(new Translation2d(-xSpeed, ySpeed), rotValue, false);

          } else { // No target
            //m_vision.setRightLLAsPrimary(true);
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
        xAtTolerance = atTolerance(TX, TX_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[0], "tx");
        yAtTolerance = atTolerance(TY, TY_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[1], "ty");
        rotAtTolerance = atTolerance(ROT, ROT_SETPOINT, LimelightConstants.REEF_TOLERANCE_ALIGNMENT[2], "rot");

    if (xAtTolerance && yAtTolerance && rotAtTolerance)
    {   m_vision.xController.reset();
        m_vision.yController.reset();
        m_vision.rotController.reset();
        //this.m_vision.enabledLEDS(false);
        System.out.println("Ended alignment within tolerance");
        return true;
    }

    if(this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME)) {
        //this.m_vision.enabledLEDS(false);
        System.out.println("Ended alignment on timeout");
        return true;
    } else {
        return false;
    }
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
  
   public boolean atTolerance(double current_pos, double set_position, double tolerance, String label)
   {
    
    double difference = current_pos - set_position;

    if(LimelightConstants.DEBUG_ENABLED){
        SmartDashboard.putNumber("$[Align]: " + label, Math.abs(difference));
        SmartDashboard.putBoolean("$[At Tolerance]: " + label, Math.abs(difference) < tolerance);
    }
    if (Math.abs(difference) > tolerance)
    {
        return false;
    }
    return true;
   }

}

