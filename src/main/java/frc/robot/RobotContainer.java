// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /* --------------------- SWERVE INIT ---------------------------- */

  // Change to correct drive base configuration 
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  AbsoluteDriveAdv closAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,() -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                              () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                OperatorConstants.DEADBAND),
                                                              () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                m_driverController.getHID()::getYButtonPressed,
                                                                m_driverController.getHID()::getAButtonPressed,
                                                                m_driverController.getHID()::getXButtonPressed,
                                                                m_driverController.getHID()::getBButtonPressed);
                                                              
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() ,
                                                                () -> m_driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> MathUtil.applyDeadband(m_driverController.getRightX() *-1,OperatorConstants.DEADBAND))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                            m_driverController::getRightY)
                          .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle); 
  
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

                                                                     
  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  //Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /* --------------------- SWERVE INTIT END ---------------------------- */

  ElevatorSubsystem elevator = new ElevatorSubsystem();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    elevator.resetEncoders();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
    driveFieldOrientedAnglularVelocity :
    driveFieldOrientedDirectAngleSim);

    if (Robot.isSimulation())
    {
    m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    m_driverController.b().whileTrue(drivebase.sysIdDriveMotorCommand());
    m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    m_driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    m_driverController.back().whileTrue(drivebase.centerModulesCommand());
    m_driverController.leftBumper().onTrue(Commands.none());
    m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
    m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    m_driverController.b().onTrue(Commands.none());
    m_driverController.y().whileTrue(Commands.none());
    m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().whileTrue(Commands.none());
    m_driverController.leftBumper().whileTrue(Commands.none());
    m_driverController.rightBumper().onTrue(Commands.none());
}
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
