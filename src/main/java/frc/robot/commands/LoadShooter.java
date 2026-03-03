// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandoffSubsystem;
import frc.robot.subsystems.SpindexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadShooter extends Command {
  private SpindexSubsystem spindex;
  private HandoffSubsystem handoff;
  /** Creates a new LoadShooter. */
  public LoadShooter(SpindexSubsystem spindex_, HandoffSubsystem handoff_) {
    this.spindex = spindex_;
    this.handoff = handoff_;
    addRequirements(this.spindex, this.handoff);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.spindex.spindexMotor.set(-1);
    this.handoff.shooterHandoffMotor.set(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.spindex.spindexMotor.set(0);
    this.handoff.shooterHandoffMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}