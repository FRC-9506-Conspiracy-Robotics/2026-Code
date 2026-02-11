// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimelightVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Num;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  private int closest_target_point = -1; // -1 means did not find any targets in range
  private double [][] target_points = {
    {999, 999, 0}, // target_points[0][0] gives the first number in this row, target_points[0][1] gives the second number in this row
    {1.905, 3.86, 0}, // target_points[1]
    {888, 888, 0}} // target_points[2]
    // table should be {x-coordinate, y-coordinate, yaw}
  ;
  private boolean tagFound = false;
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
    closest_target_point = -1;
    tagFound = false; // This resets the tagFound and closest_target_point when run
    
    if ( LimelightHelpers.getTV("") == true ) {
      System.out.print("Target Found");
      tagFound = true;
      PoseEstimate currentPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
      double minimum_distance = 99999;
      for (int i = 0; i < target_points.length; i++) {
        double posX = currentPos.pose.getMeasureX().in(Meters);
        double posY = currentPos.pose.getMeasureY().in(Meters);
        double xOffset = posX - target_points[i][0];
        double yOffset = posY - target_points[i][1];
        double distance_to_target = Math.sqrt((xOffset * xOffset) + (yOffset * yOffset));
        if (distance_to_target < minimum_distance) {
          minimum_distance = distance_to_target;
          closest_target_point = i;
        }
      };

    }
    else {
      System.out.print("Target Not Found");
    };

  //   current_position = get robot current position
  //   minimum_distance = infinity;
    
  //   for each target_point {
  //     distance_to_target = distance between current_position and target_point.position;

  //     if ((distance_to_target < minimum_distance) && i_see_valid_targets)  {
  //       minimum_distance = distance_to_target;
  //       closest_target = target_point;
  //     }
  //   }

     }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    double robotAngle = mt2Pose.pose.getRotation().getDegrees();
    double xDisplacement = mt2Pose.pose.getMeasureX().in(Meters);
    double yDisplacement = mt2Pose.pose.getMeasureY().in(Meters);
    // System.out.printf("Robot Yaw is: %f\n", robotAngle);

    double xMeters_from_zone = target_points[closest_target_point][0] - xDisplacement;
    double yMeters_from_zone = target_points[closest_target_point][1] - yDisplacement;
    double yaw_difference = target_points[closest_target_point][2] - robotAngle;

    if (xMeters_from_zone > 1) {
      xMeters_from_zone = 1;
    }
    else if (xMeters_from_zone < -1) {
      xMeters_from_zone = -1;
    }

    if (yMeters_from_zone > 1) {
      yMeters_from_zone = 1;
    }
    else if (yMeters_from_zone < -1) {
      yMeters_from_zone = -1;
    }

    if (yaw_difference > 1) {
      yaw_difference = 1;
    }
    else if (yaw_difference < -1) {
      yaw_difference = -1;
    }

    double xError = xMeters_from_zone * MaxSpeed/2;
    double yError = yMeters_from_zone * MaxSpeed/2;
    double yawError = yaw_difference * MaxAngularRate;

    this.xAxisError.set(xError);
    this.yAxisError.set(yError);

    if ( LimelightHelpers.getTV("") == true ) {
    SwerveRequest request = drive.withVelocityX(xError)
    .withVelocityY(yError)
    .withRotationalRate(yawError); 
    this.swerdrive.setControl(request);   
    }
    else {
      SwerveRequest request = drive.withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0); 
    this.swerdrive.setControl(request); 
    }


   
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
