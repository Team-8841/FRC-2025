package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.filter.MedianFilter;


public class MoveToApril extends Command {

    private Vision m_vision; 
    private SwerveSubsystem m_drive;
    private double[] leftCoralOffset = LimelightConstants.RIGHT_CORAL_OFFSETS;
    private double[] rightCoralOffset = LimelightConstants.LEFT_CORAL_OFFSETS;
    private Timer dontSeeTagTimer, stopTimer;
    private PIDController xController, yController, rotController;
    private double tagID = -1;
    private MedianFilter filter = new MedianFilter(6);

    public MoveToApril(Vision m_vision, SwerveSubsystem m_drive, boolean toRight) // 0:Left, 1:Right
    {
        this.m_vision = m_vision;
        this.m_drive = m_drive;

        xController = new PIDController(LimelightConstants.X_REEF_ALIGNMENT_P, 0.0, LimelightConstants.REEF_ALIGNMENT_D);  // Vertical movement
        yController = new PIDController(LimelightConstants.Y_REEF_ALIGNMENT_P, 0.0, LimelightConstants.REEF_ALIGNMENT_D);  // Horitontal movement
        rotController = new PIDController(LimelightConstants.ROT_REEF_ALIGNMENT_P, 0, LimelightConstants.REEF_ALIGNMENT_D);  // Rotation
        
        if (toRight == true)
        { // Target Right Reef
            m_vision.setTargetCoralToRight();
            setPIDSetpoints(LimelightConstants.RIGHT_CORAL_OFFSETS, LimelightConstants.REEF_TOLERANCE_ALIGNMENT);
        }
        else
        { // Target Left Reef
            m_vision.setTargetCoralToLeft();
            setPIDSetpoints(LimelightConstants.LEFT_CORAL_OFFSETS, LimelightConstants.REEF_TOLERANCE_ALIGNMENT);
        }

        
        this.addRequirements(this.m_drive);
    }


    @Override
    public void initialize()
    {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        

        tagID = LimelightHelpers.getFiducialID(m_vision.getLLName());
    }

    @Override
    public void execute()
    {
        //double[] current_location = m_vision.getTargetpose_Robotspace();
        if (LimelightHelpers.getTV(m_vision.getLLName()) && LimelightHelpers.getFiducialID(m_vision.getLLName()) == tagID) {
            this.dontSeeTagTimer.reset();
      
            double[] postions = LimelightHelpers.getBotPose_TargetSpace(m_vision.getLLName());
            SmartDashboard.putNumber("x", postions[2]);
      
            double xSpeed = xController.calculate(filter.calculate(postions[2]));
            double ySpeed = -yController.calculate(filter.calculate(postions[0]));
            double rotValue = -rotController.calculate(filter.calculate(postions[4]));

            SmartDashboard.putNumber("xspee", xSpeed);
            SmartDashboard.putNumber("yspee", ySpeed);
            SmartDashboard.putNumber("rotValue", rotValue);
      
            m_drive.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
              stopTimer.reset();
            }
          } else {
            m_drive.drive(new Translation2d(0,0), 0, false);
          }
      
          SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
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
    if (this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME) ||
    stopTimer.hasElapsed(LimelightConstants.POSE_VALIDATION_TIME)){
        System.out.println("FinishedCommand");
    }
    
    return this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LimelightConstants.POSE_VALIDATION_TIME);
   }


   public void setPIDSetpoints(double[] offset, double[] tolerance)
   {
   
       xController.setSetpoint(offset[0]); // X
       xController.setTolerance(tolerance[0]);

       yController.setSetpoint(offset[1]); // Y
       yController.setTolerance(tolerance[1]);

       rotController.setSetpoint(offset[2]); // Z
       rotController.setTolerance(tolerance[2]);
   }
  
    
}

