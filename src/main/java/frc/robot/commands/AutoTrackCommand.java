// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
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
  final DoublePublisher feedforwardData;
  final DoublePublisher velocityData;
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
    this.feedforwardData = table.getDoubleTopic("shooter-signals/feedforward-signal").publish();
    this.velocityData = table.getDoubleTopic("shooter-signals/velocity-signal").publish();
  }

  private class VelocityVector {
    private double velX;
    private double velY;
  }

  private double getVelocity(double angle, double d) {
    double g = 9.81;
    double h = 1.8;
    return Math.sqrt((-g * d * d) / (2 * Math.cos(angle) * Math.cos(angle) * (h - d * Math.tan(angle))));
  }

  private VelocityVector getVelVector(double velocity, double xToHub, double yToHub, double robotVelX, double robotVelY, double angler) {
    
    double fieldVelocity = velocity * Math.cos(angler);
    double angleToHub = (Math.atan2(yToHub, xToHub));
    VelocityVector staticVector = new VelocityVector();
    staticVector.velX = fieldVelocity * Math.cos(angleToHub);
    staticVector.velY = fieldVelocity * Math.sin(angleToHub);
    VelocityVector adjustedVector = new VelocityVector();
    adjustedVector.velX = staticVector.velX - robotVelX * 1.15;
    adjustedVector.velY = staticVector.velY - robotVelY * 1.15;
    return adjustedVector;
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
      IntakeSubsystem.outOfZone = false;
    }
    else if (p.x > 5.2 && p.y > 4.1) {
      targetX = 2.5;
      targetY = 6;
      IntakeSubsystem.outOfZone = true;
    }
    else if (p.x > 5.2 && p.y < 3.9) {
      targetX = 2.5;
      targetY = 2;
      IntakeSubsystem.outOfZone = true;
    }

    
    double robotAngle = p.yaw;
    double robotAngleOffset = 46.55 * (Math.PI / 180);
    double turretOffsetX = 0.158 * Math.cos((robotAngle * (Math.PI/180)) + robotAngleOffset);
    double turretOffsetY = 0.158 * Math.sin((robotAngle * (Math.PI/180)) + robotAngleOffset);
    double xFromHub = (targetX - turretOffsetX) - p.x;
    double yFromHub = (targetY - turretOffsetY) - p.y;
    double distanceFromTarget = Math.sqrt((xFromHub * xFromHub) + (yFromHub * yFromHub));
    double desiredAnglerAngle = (83 - (distanceFromTarget * 4)) * (Math.PI / 180);

    // get angle in radians
    double currentAnglerAngle = (75 - (this.turret.anglerMotor.getPosition().refresh().getValueAsDouble() / TurretConstants.anglerGearRatio) * 360) * (Math.PI / 180);
    
    


    if (desiredAnglerAngle < (50 * (Math.PI / 180))) {
      desiredAnglerAngle = (50 * (Math.PI / 180));
    }
    else if (desiredAnglerAngle > (75 * (Math.PI / 180))) {
      desiredAnglerAngle = (75 * (Math.PI / 180));
    }

    if ((p.x > 3.6 && p.x < 6.1) || (p.x < 13 && p.x > 11)) {
      desiredAnglerAngle = (75 * (Math.PI / 180));
    }
    
    VelocityVector newVelVector = getVelVector(getVelocity(desiredAnglerAngle, distanceFromTarget), xFromHub, yFromHub, p.velX, p.velY, desiredAnglerAngle);

    double distanceFromAimingPoint = Math.sqrt((Math.pow(newVelVector.velY, 2) + Math.pow(newVelVector.velX, 2)));

    double powerScaled = 1;
    // if (distanceFromAimingPoint > 2.3) {
    //   powerScaled = 1 + (Math.pow((distanceFromAimingPoint - 2.3) * 0.2 + (0.01 * (distanceFromAimingPoint - 2.3)), 2));
    // }

    double currentRotation = this.turret.getNeckPosition() + this.turret.turretOffset / 360;    
    double pastHalfPoint = (currentRotation > 0.5) ? 2 : 0;
    double desiredRotation = ((robotAngle - (Math.atan2(newVelVector.velY * powerScaled, newVelVector.velX * powerScaled) * (180 / Math.PI)) + 90) / 360) + pastHalfPoint;
    
    double newVel = Math.sqrt(Math.pow(newVelVector.velX * powerScaled, 2) + Math.pow(newVelVector.velY * powerScaled, 2)) / Math.cos(desiredAnglerAngle);

    

    
    
    if (desiredRotation < (-1.028)) {
      desiredRotation += 2;
    }
    else if (desiredRotation < (-0.028)) {
      desiredRotation += 1;
    }
    else if (desiredRotation > (2.028)) {
      desiredRotation += -2;
    }
    else if (desiredRotation > (1.028)) {
      desiredRotation += -1;
    }

    double rotationError = desiredRotation - currentRotation;
    double kP = 5;
    double controlSignal = kP * rotationError;
    if (controlSignal > 0.85) {
      controlSignal = 0.85;
    }
    else if (controlSignal < -0.85) {
      controlSignal = -0.85;
    }

   this.turret.neckMotor.set(controlSignal);

    double anglerError = desiredAnglerAngle - currentAnglerAngle;
    double anglerkP = 1.5;
    double anglerSignal = anglerkP * -anglerError;
    if (anglerSignal > 0.4) {
      anglerSignal = 0.4;
    }
    else if (anglerSignal < -0.4) {
      anglerSignal = -0.4;
    }

   this.turret.anglerMotor.set(anglerSignal);

  this.currentRotData.set(currentRotation);
  this.desiredRotData.set(desiredRotation);
  this.poseX.set(p.x);
  this.poseY.set(p.y);
  this.poseYaw.set(p.yaw);

  double shooterTuning = 1.7;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.shooterkS, TurretConstants.shooterkV, TurretConstants.shooterkA);
  double feedforwardSignal = feedforward.calculate(newVel * shooterTuning / TurretConstants.circumferenceOfWheel);
  

  double velocityError = (newVel * shooterTuning / TurretConstants.circumferenceOfWheel) - this.turret.shooterLeaderMotor.getVelocity().refresh().getValueAsDouble();
  double velocitykP = 0.01;
  double velocitySignal = velocityError * velocitykP;

  if (!TurretSubsystem.active) {
    feedforwardSignal = 0;
    velocitySignal = 0;
  }

double volts = feedforwardSignal;

this.feedforwardData.set(feedforwardSignal);
this.velocityData.set(velocitySignal);

if (volts > 12) {
  volts = 12;
}
else if (volts < -12) {
  volts = -12;
}

double distanceFactor = Math.pow(1 + (distanceFromAimingPoint - 2.7) * 0.15, Math.pow(1 + (distanceFromAimingPoint - 2.7), 2));
// if (distanceFactor < 1) {
//   distanceFactor = Math.pow(distanceFactor, 3.42);
// }
// else if (distanceFactor > 1) {
//   distanceFactor = Math.pow(distanceFactor, 2.225);
// }

double movingFactor = 1;
if (p.velX > 0.1 || p.velX < -0.1 || p.velY > 0.1 || p.velY < -0.1) {
  movingFactor = 1 + (Math.sqrt((Math.pow(p.velX, 2) + Math.pow(p.velY, 2))) * 0.3);
}
  
  this.turret.shooterLeaderMotor.setVoltage(volts * distanceFactor * movingFactor);

  // BangBangController bb_Controller = new BangBangController();

  // if (TurretSubsystem.active) {
  //   this.turret.shooterLeaderMotor.set(bb_Controller.calculate(this.turret.shooterLeaderMotor.getVelocity().refresh().getValueAsDouble(), newVel / TurretConstants.circumferenceOfWheel));
  // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turret.neckMotor.set(0);
    this.turret.anglerMotor.set(0);
    this.turret.shooterLeaderMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}