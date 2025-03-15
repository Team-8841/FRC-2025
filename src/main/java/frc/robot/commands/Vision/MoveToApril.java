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


public class MoveToApril extends Command {

    private Vision m_vision; 
    private SwerveSubsystem m_drive;
    private double[] leftCoralOffset = LimelightConstants.RIGHT_CORAL_OFFSETS;
    private double[] rightCoralOffset = LimelightConstants.LEFT_CORAL_OFFSETS;
    private Timer dontSeeTagTimer, stopTimer;
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private SwerveSubsystem drivebase;
    private double tagID = -1;

    public MoveToApril(Vision m_vision, SwerveSubsystem m_drive, boolean toRight) // 0:Left, 1:Right
    {
        this.m_vision = m_vision;
        this.m_drive = m_drive;

        if (toRight == true)
        {
            m_vision.setTargetCoralToRight();
        }
        else
        {
            m_vision.setTargetCoralToLeft();
        }

        this.addRequirements(this.m_vision, this.m_drive);
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


    @Override
    public void initialize()
    {
        if (m_vision.isTargetRightCoral() == true)
        {
            // Target Right Coral
            setPIDSetpoints(LimelightConstants.RIGHT_CORAL_OFFSETS, LimelightConstants.REEF_TOLERANCE_ALIGNMENT);
        }
        else
        {
            // Target Left Coral
            setPIDSetpoints(LimelightConstants.LEFT_CORAL_OFFSETS, LimelightConstants.REEF_TOLERANCE_ALIGNMENT);
        }

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
      
            double xSpeed = xController.calculate(postions[2]);
            SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = -rotController.calculate(postions[4]);
      
            drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
              stopTimer.reset();
            }
          } else {
            drivebase.drive(new Translation2d(), 0, false);
          }
      
          SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted)
   {
        drivebase.drive(new Translation2d(), 0, false);
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished()
   {
    return this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LimelightConstants.POSE_VALIDATION_TIME);
   }
  
    
}

