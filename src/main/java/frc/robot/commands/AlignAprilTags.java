// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimelightVisionSubsystem;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAprilTags extends Command {
  // Create properties for limelight subsystem and drivetrain
  private LimelightVisionSubsystem limelight;
  private CommandSwerveDrivetrain swerdrive;

  /** Creates a new AlignAprilTags. */
  public AlignAprilTags(
    LimelightVisionSubsystem limelight,
    CommandSwerveDrivetrain drivetrain
  ) {
    this.limelight = limelight;
    this.swerdrive = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.SetRobotOrientation(
      "", 
      this.swerdrive.getRotation3d().getAngle(), 
      0, 0, 0, 0, 0);
    PoseEstimate currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    System.out.printf("Robot X is: %f\n", mt2Pose.pose.getMeasureX().in(Units.Meters));
    System.out.printf("Robot Y is: %f\n", mt2Pose.pose.getMeasureY().in(Units.Meters));
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
