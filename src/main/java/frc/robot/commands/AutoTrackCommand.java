// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.PositionData.Pose;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTrackCommand extends Command {
  final DoublePublisher desiredRotData;
  final DoublePublisher currentRotData;
  private TurretSubsystem turret;
  private PositionData positionData;
  /** Creates a new AutoTrackCommand. */
  public AutoTrackCommand(TurretSubsystem turret_, PositionData positionData_) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret_;
    this.positionData = positionData_;
    addRequirements(this.turret);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.desiredRotData = table.getDoubleTopic("turret-auto-track/desired-rotation").publish();
    this.currentRotData = table.getDoubleTopic("turret-auto-track/current-rotation").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose p = this.positionData.getPoseData();
    double robotAngle = p.yaw;
    double xFromHub = 4.83 - p.x;
    double yFromHub = 4 - p.y;
    double distanceFromHub = Math.sqrt((xFromHub * xFromHub) + (yFromHub * yFromHub));

    double desiredRotation = ((Math.atan2(yFromHub, -xFromHub) * (180 / Math.PI)) + robotAngle - 90) / 360;
    double currentRotation = this.turret.getNeckPosition();
    

    if (desiredRotation < 0) {
      desiredRotation += 1;
    }
    else if (desiredRotation > 1) {
      desiredRotation += -1;
    }

    double rotationError = desiredRotation - currentRotation;
    double kP = 2;
    double controlSignal = kP * rotationError;
    if (controlSignal > 1) {
      controlSignal = 1;
    }
    else if (controlSignal < -1) {
      controlSignal = -1;
    }

    this.turret.neckMotor.set(controlSignal);

  this.currentRotData.set(currentRotation);
  this.desiredRotData.set(desiredRotation);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.shooterkS, TurretConstants.shooterkV, TurretConstants.shooterkA);
  double volts = feedforward.calculate(distanceFromHub * 0.75, 0);
  if (volts > 12) {
    volts = 12;
  }
  this.turret.shooterLeaderMotor.setVoltage(volts);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
