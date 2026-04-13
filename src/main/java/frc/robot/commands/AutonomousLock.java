// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PositionData.Pose;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousLock extends Command {
  private SwerveSubsystem swerve;
  private PositionData positionData;
  /** Creates a new AutonomousLock. */
  public AutonomousLock(SwerveSubsystem swerve_, PositionData positionData_) {
    this.swerve = swerve_;
    this.positionData = positionData_;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose pose = positionData.getPose();
    this.swerve.driveToPose(new Pose2d(pose.x, pose.y, new Rotation2d(Math.atan2(4 - pose.y, 4.6 - pose.x))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
