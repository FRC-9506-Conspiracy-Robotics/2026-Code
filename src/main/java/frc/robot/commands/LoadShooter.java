// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandoffSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadShooter extends Command {
  private HopperSubsystem hopper;
  private HandoffSubsystem handoff;
  private IntakeSubsystem intake;
  /** Creates a new LoadShooter. */
  public LoadShooter(HopperSubsystem hopper_, HandoffSubsystem handoff_, IntakeSubsystem intake_) {
    this.hopper = hopper_;
    this.handoff = handoff_;
    this.intake = intake_;
    addRequirements(this.hopper, this.handoff, this.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.hopper.hopperMotor.set(-1);
    this.handoff.shooterHandoffMotor.set(0.75);
    if (this.intake.deployEncoder.getPosition() < -4.5) {
      this.intake.intakeMotor.set(-0.75);
    }
    else {
      this.intake.intakeMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.hopper.hopperMotor.set(0);
    this.handoff.shooterHandoffMotor.set(0);
    this.intake.intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
