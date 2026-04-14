// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AutonomousLock;
import frc.robot.commands.LoadShooter;
import frc.robot.commands.LockOn;
import frc.robot.commands.LockPose;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.DrumShooterSubsystem;
import frc.robot.subsystems.HubCounter;
import frc.robot.subsystems.HubShiftUtil;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final SwerveDrive swerveDrive = drivebase.getSwerveDrive();
  public static boolean lockOn = false;
  //converts controller inputs to swerveinputstream type for field oriented
  SwerveInputStream driveAngularVelocity = 
  SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -mDriverController.getLeftY() * PositionData.speedFactor,
      () -> -mDriverController.getLeftX() * PositionData.speedFactor
  )
  .withControllerRotationAxis(() -> lockOn ? PositionData.rotationSignal : -mDriverController.getRightX())
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
  () -> -mDriverController.getLeftY() * PositionData.speedFactor,
  () -> -mDriverController.getLeftX() * PositionData.speedFactor)
  .withControllerRotationAxis(
      () -> mDriverController.getRawAxis(2)
  )
  .deadband(DriverConstants.kDeadband)
  .scaleTranslation(0.8)
  .allianceRelativeControl(false);

  private final SendableChooser<Command> autoChooser;

  private HubCounter hubCounter = new HubCounter();

  public final PositionData positionData = new PositionData(swerveDrive);

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DrumShooterSubsystem drumShooter = new DrumShooterSubsystem(positionData);
  private final AnglerSubsystem angler = new AnglerSubsystem(positionData);

  private final LoadShooter loadShooter = new LoadShooter(drumShooter, intake);
  private final LockOn lock = new LockOn();
  private final AutonomousLock autoLock = new AutonomousLock(drivebase, positionData);
  private final LockPose lockPose = new LockPose(swerveDrive);

  private SwerveDriveSimulation swerveDriveSimulation;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case SIM:
        SimulatedArena.getInstance();
        final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(COTS.ofMark4(
            DCMotor.getKrakenX60(1), 
            DCMotor.getKrakenX44(1), 
              1.19, 
              3));
      
        this.swerveDriveSimulation = new SwerveDriveSimulation(
          driveTrainSimulationConfig, 
          new Pose2d(3, 4, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        
    }

    NamedCommands.registerCommand("Deploy Intake", this.intake.toggleDeploy()); // uses intake
    NamedCommands.registerCommand("Lock On", this.autoLock);
    NamedCommands.registerCommand("Toggle Shooter", this.drumShooter.toggleShooter());
    NamedCommands.registerCommand("Load Shooter", this.loadShooter);
    
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(hubCounter::initialize));

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

    // this.turret.setDefaultCommand(autoTrackCommand);

    mDriverController.x().onTrue(this.intake.toggleReload());
    mDriverController.y().onTrue(this.intake.toggleDeploy());
    mDriverController.a().whileTrue(lockPose);
    mDriverController.b().onTrue(this.drumShooter.toggleShooter());
    mDriverController.leftBumper().whileTrue(this.intake.unjamIntake());
    mDriverController.leftTrigger().whileTrue(this.lock);
    mDriverController.rightBumper().whileTrue(this.drumShooter.unjamShooter());
    mDriverController.rightTrigger().whileTrue(loadShooter);

    mDriverController.start().and(mDriverController.povRight()).whileTrue(this.drumShooter.sysIdQuasistatic(Direction.kForward));
    mDriverController.start().and(mDriverController.povLeft()).whileTrue(this.drumShooter.sysIdQuasistatic(Direction.kReverse));
    mDriverController.back().and(mDriverController.povRight()).whileTrue(this.drumShooter.sysIdDynamic(Direction.kForward));
    mDriverController.back().and(mDriverController.povLeft()).whileTrue(this.drumShooter.sysIdDynamic(Direction.kReverse));

    

  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


}