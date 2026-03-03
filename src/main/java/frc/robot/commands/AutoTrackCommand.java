// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.PositionData.Pose;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTrackCommand extends Command {
  double targetX = 4.6;
  double targetY = 4;

  final DoublePublisher desiredRotData;
  final DoublePublisher currentRotData;
  final DoublePublisher poseX;
  final DoublePublisher poseY;
  final DoublePublisher poseYaw;
  private TurretSubsystem turret;
  private PositionData positionData;
  /** Creates a new AutoTrackCommand. */
  public AutoTrackCommand(TurretSubsystem turret_, PositionData positionData_) { //min - max for angler is 15 - 45
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret_;
    this.positionData = positionData_;
    addRequirements(this.turret);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.desiredRotData = table.getDoubleTopic("turret-auto-track/desired-rotation").publish();
    this.currentRotData = table.getDoubleTopic("turret-auto-track/current-rotation").publish();
    this.poseX = table.getDoubleTopic("turret-auto-track/px").publish();
    this.poseY = table.getDoubleTopic("turret-auto-track/py").publish();
    this.poseYaw = table.getDoubleTopic("turret-auto-track/pyaw").publish();
  }

  private double getVelocity(double angle, double d) {
    double g = 9.81;
    double h = 1.778;
    return Math.sqrt((-g * d * d) / (2 * Math.cos(angle) * Math.cos(angle) * (h - d * Math.tan(angle))));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose p = this.positionData.getPose();

    if (p.x < 4.6) {
      targetX = 4.6;
      targetY = 4;
    }
    else if (p.x > 5.2 && p.y > 4.1) {
      targetX = 4.6;
      targetY = 6;
    }
    else if (p.x > 5.2 && p.y < 3.9) {
      targetX = 4.6;
      targetY = 2;
    }

    
    double robotAngle = p.yaw;
    double xFromHub = targetX - p.x;
    double yFromHub = targetY - p.y;
    double distanceFromTarget = Math.sqrt((xFromHub * xFromHub) + (yFromHub * yFromHub));

    // get angle in radians
    double currentAnglerAngle = (75 - (this.turret.anglerMotor.getPosition().refresh().getValueAsDouble() / TurretConstants.anglerGearRatio) * 360) * (Math.PI / 180);
    
    double desiredAnglerAngle = (83 - (distanceFromTarget * 4)) * (Math.PI / 180);

    if (desiredAnglerAngle < (50 * (Math.PI / 180))) {
      desiredAnglerAngle = (50 * (Math.PI / 180));
    }
    else if (desiredAnglerAngle > (75 * (Math.PI / 180))) {
      desiredAnglerAngle = (75 * (Math.PI / 180));
    }

    double desiredRotation = ((Math.atan2(yFromHub, -xFromHub) * (180 / Math.PI)) + robotAngle - 90) / 360;
    double currentRotation = this.turret.getNeckPosition();
    
    

    if (desiredRotation < -0.012) {
      desiredRotation += 1;
    }
    else if (desiredRotation > 1.012) {
      desiredRotation += -1;
    }

    double rotationError = desiredRotation - currentRotation;
    double kP = 4;
    double controlSignal = kP * rotationError;
    if (controlSignal > 0.5) {
      controlSignal = 0.5;
    }
    else if (controlSignal < -0.5) {
      controlSignal = -0.5;
    }

   this.turret.neckMotor.set(controlSignal);

    double anglerError = desiredAnglerAngle - currentAnglerAngle;
    double anglerkP = 1.5;
    double anglerSignal = anglerkP * -anglerError;
    if (anglerSignal > 1) {
      anglerSignal = 1;
    }
    else if (anglerSignal < -1) {
      anglerSignal = -1;
    }

   this.turret.anglerMotor.set(anglerSignal);

  this.currentRotData.set(currentRotation);
  this.desiredRotData.set(desiredRotation);
  this.poseX.set(p.x);
  this.poseY.set(p.y);
  this.poseYaw.set(p.yaw);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.shooterkS, TurretConstants.shooterkV, TurretConstants.shooterkA);
  double volts = feedforward.calculate(getVelocity(currentAnglerAngle, distanceFromTarget) * 1.7 / TurretConstants.circumferenceOfWheel);
  if (volts > 12) {
    volts = 12;
  }

  if (!TurretSubsystem.active) {
    volts = 0;
  }

  this.turret.shooterLeaderMotor.setVoltage(volts);

  // BangBangController bb_Controller = new BangBangController();
  // this.turret.shooterLeaderMotor.set(bb_Controller.calculate(this.turret.shooterLeaderMotor.getVelocity().refresh().getValueAsDouble(), getVelocity(currentAnglerAngle, distanceFromTarget) / TurretConstants.circumferenceOfWheel));

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