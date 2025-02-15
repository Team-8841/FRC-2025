// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climber.MoveClimberToPosition;
import frc.robot.commands.Elevator.MoveToSetpoint;
import frc.robot.commands.Gripper.IntakeSensorControl;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import org.photonvision.PhotonCamera;

import swervelib.*;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_copilotController = 
  new CommandJoystick(OperatorConstants.kCoPilotControllerPort);


  private PhotonCamera mainCam = new PhotonCamera("center");

  /* --------------------- SWERVE INIT ---------------------------- */

  // Change to correct drive base configuration 
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final Gripper m_Gripper = new Gripper();
  private final Climber m_Climber = new Climber();

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

    m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    m_driverController.back().whileTrue(drivebase.centerModulesCommand());
    m_driverController.leftBumper().onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.startingConfiguration));
    m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
    m_driverController.a().onTrue(new MoveClimberToPosition(m_Climber, ClimberConstants.CLIMBER_HOME_POSITION));
    m_driverController.x().onTrue(new MoveClimberToPosition(m_Climber, ClimberConstants.CLIMBER_RETRACT_POSITION));
    m_driverController.y().onTrue(new MoveClimberToPosition(m_Climber, ClimberConstants.CLIMBER_DEPLOY_POSITION));
    m_driverController.b().onTrue(Commands.none());
    m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().whileTrue(Commands.none());
    m_driverController.leftBumper().whileTrue(new MoveToSetpoint(elevator, m_Gripper, SetpointConstants.groundPickup));
    m_driverController.rightBumper().whileTrue(Commands.none());

    m_copilotController.button(OperatorConstants.CoralL1).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL1));
    m_copilotController.button(OperatorConstants.CoralL2).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL2));
    m_copilotController.button(OperatorConstants.CoralL3).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL3));
    m_copilotController.button(OperatorConstants.CoralL4).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL4));

    m_copilotController.button(OperatorConstants.AlgaeL1).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.AlgaeL1));
    m_copilotController.button(OperatorConstants.AlgaeL2).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.AlgaeL2));
    m_copilotController.button(OperatorConstants.AlgaeL3).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.AlgaeL3));
    m_copilotController.button(OperatorConstants.AlgaeL4).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.AlgaeL4));

    m_copilotController.button(OperatorConstants.IntakeIn).whileTrue(new RunCommand( () -> {
      m_Gripper.setGripperSpeed(-.75);
    }, m_Gripper)).onFalse(new InstantCommand(() -> {
      m_Gripper.setGripperSpeed(0);
    }));
    m_copilotController.button(OperatorConstants.IntakeOut).whileTrue(new RunCommand( () -> {
      m_Gripper.setGripperSpeed(0.75);
    }, m_Gripper)).onFalse(new InstantCommand(() -> {
      m_Gripper.setGripperSpeed(0);
    }));
    //m_copilotController.button(OperatorConstants.IntakeIn).whileTrue(new IntakeSensorControl(m_Gripper, 0.5)).onFalse(new IntakeSensorControl(m_Gripper, 0));
    //m_copilotController.button(OperatorConstants.IntakeOut).whileTrue(new IntakeSensorControl(m_Gripper, -0.5)).onFalse(new IntakeSensorControl(m_Gripper, 0));

    m_copilotController.button(OperatorConstants.ManualOverride).whileTrue(Commands.none());


    
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
