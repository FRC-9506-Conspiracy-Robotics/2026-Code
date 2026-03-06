// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AutoTrackCommand;
import frc.robot.commands.LoadShooter;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HandoffSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.SpindexSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final SwerveDrive swerveDrive = drivebase.getSwerveDrive();

  //converts controller inputs to swerveinputstream type for field oriented
  SwerveInputStream driveAngularVelocity = 
  SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -mDriverController.getLeftY(),
      () -> -mDriverController.getLeftX()
  )
  .withControllerRotationAxis(() -> -mDriverController.getRightX())
  .deadband(DriverConstants.kDeadband)
  .scaleTranslation(0.8)
  .allianceRelativeControl(false);
  
  //copies previous stream and converts to robot oriented
  SwerveInputStream driveRobotOriented = 
  driveAngularVelocity.copy()
  .robotRelative(true)
  .allianceRelativeControl(false);

  //controls for keyboard
  SwerveInputStream driveAngularVelocityKeyboard = 
  SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> -mDriverController.getLeftY(),
  () -> -mDriverController.getLeftX())
  .withControllerRotationAxis(
      () -> mDriverController.getRawAxis(2)
  )
  .deadband(DriverConstants.kDeadband)
  .scaleTranslation(0.8)
  .allianceRelativeControl(false);

  private final SendableChooser<Command> autoChooser;

  private final HandoffSubsystem handoff = new HandoffSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SpindexSubsystem spindex = new SpindexSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  public final PositionData positionData = new PositionData(swerveDrive);

  private AutoTrackCommand autoTrackCommand = new AutoTrackCommand(turret, positionData);
  private LoadShooter loadShooter = new LoadShooter(spindex, handoff);


  public RobotContainer() {

    NamedCommands.registerCommand("Load Shooter", loadShooter); // uses spindex and handoff
    NamedCommands.registerCommand("Deploy Intake", this.intake.deployIntake()); // uses intake
    NamedCommands.registerCommand("Reload", this.intake.toggleReload()); // uses intake
    NamedCommands.registerCommand("Toggle Shooter", this.turret.shooterControl()); // uses turret
    NamedCommands.registerCommand("Auto Track", autoTrackCommand);
    
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    

  }

  private void configureBindings() {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocity);
      if (RobotBase.isSimulation()) {
          drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
      } else {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }

    mDriverController.leftStick().onTrue(drivebase.zero());

    this.turret.setDefaultCommand(autoTrackCommand);

    mDriverController.rightTrigger().whileTrue(loadShooter);
    mDriverController.rightBumper().whileTrue(this.handoff.unjamShooter());
    mDriverController.x().onTrue(this.intake.toggleReload());
    mDriverController.y().onTrue(this.intake.deployIntake());
    mDriverController.leftBumper().onTrue(this.turret.shooterControl());
    mDriverController.povUp().whileTrue(this.climber.raiseClimber());
    mDriverController.povDown().whileTrue(this.climber.lowerClimber());

    

  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


}