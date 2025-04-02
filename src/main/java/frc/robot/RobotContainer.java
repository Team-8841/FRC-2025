// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerFunction;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Climber.DriveClimberWithJoystick;
import frc.robot.commands.Climber.LockClimber;
import frc.robot.commands.Climber.SetClimberLockPosition;
import frc.robot.commands.Climber.UnlockClimber;
import frc.robot.commands.Elevator.AutoMoveToFeederStation;
import frc.robot.commands.Elevator.AutoMoveToSetpointGroup;
import frc.robot.commands.Elevator.AutoMoveWristToSetpoint;
import frc.robot.commands.Elevator.MoveToHome;
import frc.robot.commands.Elevator.MoveToSetpoint;
import frc.robot.commands.Elevator.MoveToSetpointGroup;
import frc.robot.commands.Elevator.SetElevatorHomeTarget;
import frc.robot.commands.Elevator.SetElevatorTarget;
import frc.robot.commands.Gripper.AutoIntakeSensorControl;
import frc.robot.commands.Gripper.IntakeAndWait;
import frc.robot.commands.Gripper.IntakeSensorControl;
import frc.robot.commands.Gripper.ShootAlgae;
import frc.robot.commands.Gripper.StopIntake;
import frc.robot.commands.Vision.MoveToApril;
import frc.robot.commands.Vision.MoveToCenterApril;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriverCam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Lighting.AnimationTypes;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import swervelib.*;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_copilotController = 
  new CommandJoystick(OperatorConstants.kCoPilotControllerPort);

  private final SendableChooser<Command> m_autoChooser;

  /* --------------------- SWERVE INIT ---------------------------- */

  // Change to correct drive base configuration 
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final Gripper m_Gripper = new Gripper();
  private final Climber m_Climber = new Climber();
  private final Vision m_Vision = new Vision(LimelightConstants.ll_FRONT_RIGHT_NAME, LimelightConstants.ll_FRONT_LEFT_NAME); //Primary and Alternate Limelight
  private final Lighting m_Lighting = new Lighting();
  //private final DriverCam m_DriverCam = new DriverCam();

  AbsoluteDriveAdv closAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,() -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                OperatorConstants.DEADBAND),
                                                              () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                OperatorConstants.DEADBAND),
                                                              () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                OperatorConstants.DEADBAND),
                                                                m_driverController.getHID()::getYButtonPressed,
                                                                m_driverController.getHID()::getAButtonPressed,
                                                                m_driverController.getHID()::getXButtonPressed,
                                                                m_driverController.getHID()::getBButtonPressed);
  
    // AS of Feb 25 this is the used drive command                      
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> convertJoystickQuadratic(m_driverController.getLeftY()) ,
                                                                () -> convertJoystickQuadratic(m_driverController.getLeftX()))
                                                            .withControllerRotationAxis(() -> MathUtil.applyDeadband(convertJoystickQuadratic(m_driverController.getRightX() *-1),OperatorConstants.DEADBAND))
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_elevator.resetEncoders();
    m_Lighting.changeAnimation(AnimationTypes.Rainbow);

    // Auto Named Commands

    // Home positions
    NamedCommands.registerCommand("MoveToStartingConfig", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.startingConfiguration, false));
    NamedCommands.registerCommand("MoveToGroundPickup", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.groundPickup, true));
    NamedCommands.registerCommand("MoveToFeederStation", new AutoMoveToFeederStation(m_elevator, m_Gripper, SetpointConstants.feederStation, false));

    // Coral positions
    NamedCommands.registerCommand("MoveToCoralL1", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.CoralL1, false));
    NamedCommands.registerCommand("MoveToCoralL2", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.CoralL2, false));
    NamedCommands.registerCommand("MoveToCoralL3", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.CoralL3, false));
    NamedCommands.registerCommand("MoveToCoralL4", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.CoralL4, false));

    // Algae positions
    NamedCommands.registerCommand("MoveToAlgaeL1", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.AlgaeL1, true));
    NamedCommands.registerCommand("MoveToAlgaeL2", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.AlgaeL2, true));
    NamedCommands.registerCommand("MoveToAlgaeL3", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.AlgaeL3, true));
    NamedCommands.registerCommand("MoveToAlgaeL4", new AutoMoveToSetpointGroup(m_elevator, m_Gripper, SetpointConstants.AlgaeL4, true));

    // Shoot
    NamedCommands.registerCommand("Shoot", new ShootAlgae(m_Gripper, GripperConstants.IntakeShootSpeed));

    // Intake commands
    NamedCommands.registerCommand("IntakeAndWait", new IntakeAndWait(m_Gripper));
    NamedCommands.registerCommand("IntakeWithSensorControl", new AutoIntakeSensorControl(true, false, m_Gripper, m_elevator));
    NamedCommands.registerCommand("OuttakeWithSensorControl", new AutoIntakeSensorControl(false, true, m_Gripper, m_elevator));
    NamedCommands.registerCommand("StopIntake", new StopIntake(false, false, m_Gripper, m_elevator));

    // Vision Reef Auto Alignment
    NamedCommands.registerCommand("AutoAlignReefLeft", new MoveToApril(m_Vision, drivebase, false, true));
    NamedCommands.registerCommand("AutoAlignReefRight", new MoveToApril(m_Vision, drivebase, true, true));
    NamedCommands.registerCommand("AutoAlignReefCenter", new MoveToCenterApril(m_Vision, drivebase));

    NamedCommands.registerCommand("ZeroGyroBlue", new InstantCommand(() -> {
      drivebase.teleopSetup();
    }));
    
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);

  }

  private void configureBindings() {
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
    driveFieldOrientedAnglularVelocity :
    driveFieldOrientedDirectAngleSim);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    m_driverController.a().whileTrue(new MoveToCenterApril(m_Vision, drivebase));
    m_driverController.b().whileTrue(new MoveToApril(m_Vision, drivebase, true, false));
    m_driverController.x().whileTrue(new MoveToApril(m_Vision, drivebase, false, false));
    m_driverController.y().onTrue(new MoveToHome(m_elevator, m_Gripper));   // Home elevator
      

    m_driverController.start().onTrue(new InstantCommand( () -> {
      drivebase.zeroGyro();
    }));

    m_driverController.back().onTrue(new InstantCommand(() -> {
      if(DriverStation.getAlliance().get() == Alliance.Blue) {
        drivebase.teleopSetup();
      } else {
        System.out.println("Not on the blue alliancce");
      }
    }));

    //m_driverController.leftBumper().onTrue(new MoveToSetpoint(m_elevator, m_Gripper));  // Move elevator to target
    m_driverController.leftBumper().onTrue(new MoveToSetpointGroup(m_elevator, m_Gripper));  // Move elevator to target

    m_driverController.rightBumper().whileTrue(new ShootAlgae(m_Gripper, GripperConstants.IntakeShootSpeed))  // Shoot algae/coral when held down
    .onFalse(new ShootAlgae(m_Gripper, 0));

   

    m_copilotController.button(OperatorConstants.CoralL1).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL1, true, false))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL1, false, false));

    m_copilotController.button(OperatorConstants.CoralL2).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL2, true, false))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL2, false, false));

    m_copilotController.button(OperatorConstants.CoralL3).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL3, true,false))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL3, false,false));
    
    m_copilotController.button(OperatorConstants.CoralL4).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL4, true, false))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.CoralL4, false, false));

    m_copilotController.button(OperatorConstants.AlgaeL1).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL1, true, true))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL1, false, true));

    m_copilotController.button(OperatorConstants.AlgaeL2).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL2, true, true))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL2, false, true));

    m_copilotController.button(OperatorConstants.AlgaeL3).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL3, true, true))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL3, false, true));

    m_copilotController.button(OperatorConstants.AlgaeL4).onTrue(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL4, true, true))
    .onFalse(new SetElevatorTarget(m_elevator, SetpointConstants.AlgaeL4, false, true));

    m_copilotController.button(OperatorConstants.IntakeIn).whileTrue(new IntakeSensorControl(true, false, m_Gripper, m_elevator))
      .onFalse(new IntakeSensorControl(false, false, m_Gripper, m_elevator));

    m_copilotController.button(OperatorConstants.IntakeOut).whileTrue(new IntakeSensorControl(false, true, m_Gripper, m_elevator))
      .onFalse(new IntakeSensorControl(false, false, m_Gripper, m_elevator));

    m_copilotController.button(OperatorConstants.startingConfiguration).onTrue(new SetElevatorHomeTarget(m_elevator, SetpointConstants.startingHomeConfiguration, false));

    m_copilotController.button(OperatorConstants.groundPickup).onTrue(new SetElevatorHomeTarget(m_elevator, SetpointConstants.groundPickup, true));

    m_copilotController.button(OperatorConstants.feederStation).onTrue(new SetElevatorHomeTarget(m_elevator, SetpointConstants.feederStation, false));

    m_copilotController.button(OperatorConstants.ClimberLockSwitch).onTrue(new UnlockClimber(m_Climber)).onFalse(new LockClimber(m_Climber));

    m_copilotController.button(OperatorConstants.ClimberSwitch).whileTrue(new DriveClimberWithJoystick(m_copilotController, m_Climber, true))
    .onFalse(new InstantCommand(() -> {
      m_Climber.stopClimber();;
    }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   //TODO: Setup auto chooster on the smartdashboard to select between the different autonomous modes
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void resetDriveOdo() {
    drivebase.teleopSetup();
  }

  public double convertJoystickQuadratic(double input)
  {
    double db_input = Math.abs(input) - OperatorConstants.DEADBAND; // Shift curve to zero at deadband
    if (db_input < 0)
    {
      db_input = 0;
    }

    if (input < 0) {
       // Shift curves to zero at deadband
      return -((Math.pow(ControllerFunction.POWER, db_input) - 1) / (ControllerFunction.POWER - 1));
    } else {
      return (Math.pow(ControllerFunction.POWER, db_input) - 1) / (ControllerFunction.POWER - 1);
    }
  }
}
