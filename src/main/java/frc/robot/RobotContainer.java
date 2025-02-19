// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Climber.DriveClimberWithJoystick;
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


  private double curTargetPosition[] = SetpointConstants.CoralL1;
  private double curTargetHomePosition[] = SetpointConstants.startingConfiguration;
  private boolean elevatorReadyToMove = false;

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


  private void updateElevatorStatus() {
    SmartDashboard.putBoolean("Elevator Ready", getReady());
    SmartDashboard.putNumber("Elevator Target", curTargetPosition[0]);
    SmartDashboard.putNumber("Gripper Target", curTargetPosition[1]);
    SmartDashboard.putNumber("Elevator Home Target", curTargetHomePosition[0]);
    SmartDashboard.putNumber("Gripper Home Target", curTargetHomePosition[1]);
  }

  private void updateReady(boolean ready) {
    elevatorReadyToMove = ready;
  }

  private void updateElevatorTarget(double[] targets){
    curTargetPosition = targets;
  }

  private double[] getTargets() {
    return curTargetPosition;
  }

  private boolean getReady(){
    return elevatorReadyToMove;
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
    driveFieldOrientedAnglularVelocity :
    driveFieldOrientedDirectAngleSim);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    m_driverController.a().onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.groundPickup, true));
    m_driverController.b().onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.feederStation, true));
    //m_driverController.x().onTrue(new MoveToSetpoint(elevator, m_Gripper, SetpointConstants.CoralL3, true));
    //m_driverController.y().onTrue(new MoveToSetpoint(elevator, m_Gripper, SetpointConstants.CoralL4, true));
    m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().whileTrue(Commands.none());
    m_driverController.leftBumper().onTrue(new MoveToSetpoint(elevator, m_Gripper, curTargetHomePosition, true));
    m_driverController.rightBumper().onTrue(new MoveToSetpoint(elevator, m_Gripper, getTargets(), getReady()));

    m_copilotController.button(OperatorConstants.CoralL1).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL1, true));

    m_copilotController.button(OperatorConstants.CoralL2).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL2, true));

    m_copilotController.button(OperatorConstants.CoralL3).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL3, true));
    m_copilotController.button(OperatorConstants.CoralL4).onTrue(new MoveToSetpoint(m_elevator, m_Gripper, SetpointConstants.CoralL4, true));

    m_copilotController.button(OperatorConstants.AlgaeL1).onTrue(new InstantCommand(() -> {
      updateElevatorTarget(SetpointConstants.AlgaeL1);
      updateReady(true);
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      updateReady(false);
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.AlgaeL2).onTrue(new InstantCommand(() -> {
      updateElevatorTarget(SetpointConstants.AlgaeL2);
      updateReady(true);
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      updateReady(false);
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.AlgaeL3).onTrue(new InstantCommand(() -> {
      updateElevatorTarget(SetpointConstants.AlgaeL3);
      updateReady(true);
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      updateReady(false);
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.AlgaeL4).onTrue(new InstantCommand(() -> {
      updateElevatorTarget(SetpointConstants.AlgaeL4);
      updateReady(true);
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      updateReady(false);
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.IntakeIn).whileTrue(new IntakeSensorControl(true, false, m_Gripper.isCoralDetected(), m_Gripper.isAlgaeDetected(), m_Gripper))
      .onFalse(new IntakeSensorControl(false, false, m_Gripper.isCoralDetected(), m_Gripper.isAlgaeDetected(), m_Gripper));
    m_copilotController.button(OperatorConstants.IntakeOut).whileTrue(new IntakeSensorControl(false, true, m_Gripper.isCoralDetected(), m_Gripper.isAlgaeDetected(), m_Gripper))
      .onFalse(new IntakeSensorControl(false, false, m_Gripper.isCoralDetected(), m_Gripper.isAlgaeDetected(), m_Gripper));

    m_copilotController.button(OperatorConstants.feederStation).onTrue(new InstantCommand(() -> {
      curTargetHomePosition = SetpointConstants.feederStation;
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      curTargetHomePosition = SetpointConstants.startingConfiguration;
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.groundPickup).onTrue(new InstantCommand(() -> {
      curTargetHomePosition = SetpointConstants.groundPickup;
      updateElevatorStatus();
    })).onFalse(new InstantCommand(() -> {
      curTargetHomePosition = SetpointConstants.startingConfiguration;
      updateElevatorStatus();
    }));

    m_copilotController.button(OperatorConstants.ManualOverride).whileTrue(new DriveClimberWithJoystick(m_copilotController.getRawAxis(OperatorConstants.WristjoystickY), m_Climber.getOutSensorO(), m_Climber.getInSensor(), m_Climber));
    //m_copilotController.button(OperatorConstants.IntakeIn).whileTrue(new IntakeSensorControl(m_Gripper, 0.5)).onFalse(new IntakeSensorControl(m_Gripper, 0));
    //m_copilotController.button(OperatorConstants.IntakeOut).whileTrue(new IntakeSensorControl(m_Gripper, -0.5)).onFalse(new IntakeSensorControl(m_Gripper, 0));

    m_copilotController.button(OperatorConstants.ManualOverride).whileTrue(Commands.none());

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
