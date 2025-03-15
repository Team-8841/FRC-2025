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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;


public class MoveToApril extends Command {

    private Vision m_vision; 
    private SwerveSubsystem m_drive;
    private double[] leftCoralOffset = LimelightConstants.RIGHT_CORAL_OFFSETS;
    private double[] rightCoralOffset = LimelightConstants.LEFT_CORAL_OFFSETS;

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


    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        double[] current_location = m_vision.getTargetpose_Robotspace();

        //TODO calculate offset, create adjustment vector

        // Drive to left or right
    }
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted)
   {
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished()
   {
     return m_vision.visionAtSetpoint();
   }
  
    
}

