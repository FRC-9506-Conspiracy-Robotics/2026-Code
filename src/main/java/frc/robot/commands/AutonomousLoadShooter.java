// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrumShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousLoadShooter extends Command {
  private DrumShooterSubsystem drumShooter;
  private IntakeSubsystem intake;
  /** Creates a new AutonomousLoadShooter. */
  public AutonomousLoadShooter(DrumShooterSubsystem drumShooter_, IntakeSubsystem intake_) {
    this.drumShooter = drumShooter_;
    this.intake = intake_;
    addRequirements(this.drumShooter, this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.drumShooter.loading = true;
    this.intake.deploySpeed = 0.125;
    this.intake.desiredPosition = this.intake.STOWED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drumShooter.loading = false;
    this.intake.deploySpeed = 0.25;
    this.intake.desiredPosition = this.intake.DEPLOYED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
