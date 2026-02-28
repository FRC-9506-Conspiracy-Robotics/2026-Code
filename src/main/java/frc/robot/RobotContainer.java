// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //converts controller inputs to swerveinputstream type for field oriented
  SwerveInputStream driveAngularVelocity = 
  SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> mDriverController.getLeftY(),
      () -> mDriverController.getLeftX()
  )
  .withControllerRotationAxis(() -> -mDriverController.getRightX())
  .deadband(DriverConstants.kDeadband)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);
  
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
  .allianceRelativeControl(true);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    
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

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}