// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimelightVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAprilTags extends Command {
  // channel for tracking the X-axis error
  final DoublePublisher xAxisError;
  final DoublePublisher yAxisError;

  // Create properties for limelight subsystem and drivetrain
  private LimelightVisionSubsystem limelight;
  private CommandSwerveDrivetrain swerdrive;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new AlignAprilTags. */
  public AlignAprilTags(
    LimelightVisionSubsystem limelight,
    CommandSwerveDrivetrain drivetrain
  ) {
    this.limelight = limelight;
    this.swerdrive = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.xAxisError = table.getDoubleTopic("align-april-tags/x-error").publish();
    this.yAxisError = table.getDoubleTopic("align-april-tags/y-error").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.SetRobotOrientation(
      "", 
      this.swerdrive.getRotation3d().getZ(), 
      0, 0, 0, 0, 0);
    PoseEstimate currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    System.out.printf("Robot X is: %f\n", mt2Pose.pose.getMeasureX().in(Units.Meters));
    System.out.printf("Robot Y is: %f\n", mt2Pose.pose.getMeasureY().in(Units.Meters));
    double robotAngle = mt2Pose.pose.getRotation().getDegrees();
    double xDisplacement = mt2Pose.pose.getMeasureX().in(Units.Meters);
    double yDisplacement = mt2Pose.pose.getMeasureY().in(Units.Meters);
    System.out.printf("Robot Yaw is: %f\n", robotAngle);

    double xError = (1.905 - xDisplacement) * MaxSpeed/2;
    double yError = (3.86 - yDisplacement) * MaxSpeed/2;

    this.xAxisError.set(xError);
    this.yAxisError.set(yError);
    SwerveRequest request = drive.withVelocityX(xError)
    .withVelocityY(yError)
    .withRotationalRate(-robotAngle * MaxAngularRate); 
    this.swerdrive.setControl(request);   
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // Command is over when we're aligned with the AprilTAG
  @Override
  public boolean isFinished() {
    return false;
  }
}
