// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandoffSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionData;
import frc.robot.subsystems.PositionData.Pose;
import frc.robot.subsystems.SpindexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadShooter extends Command {
  private double runTime = 0;
  private double bumpPeriod = 1;
  private SpindexSubsystem spindex;
  private HandoffSubsystem handoff;
  private IntakeSubsystem intake;
  private PositionData positionData;
  /** Creates a new LoadShooter. */
  public LoadShooter(SpindexSubsystem spindex_, HandoffSubsystem handoff_, IntakeSubsystem intake_, PositionData positionData_) {
    this.spindex = spindex_;
    this.handoff = handoff_;
    this.intake = intake_;
    this.positionData = positionData_;
    addRequirements(this.spindex, this.handoff, this.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.runTime = Utils.getCurrentTimeSeconds();
    this.intake.desiredPosition = this.intake.DEPLOYED;
    this.intake.deploySpeed = 0.15;
    if (IntakeSubsystem.outOfZone == true) {
      this.bumpPeriod = 3;
    }
    else {
      this.bumpPeriod = 1;
    }
    PositionData.speedFactor = 0.29;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PositionData.speedFactor = 0.29;

    if (IntakeSubsystem.outOfZone == true) {
      this.bumpPeriod = 3;
    }
    else {
      this.bumpPeriod = 1;
    }

    Pose vector = this.positionData.getPose();
    if (vector.velX > 0.1 || vector.velX < -0.1 || vector.velY > 0.1 || vector.velY < -0.1) {
      this.bumpPeriod = 9999;
    }

    this.spindex.spindexMotor.set(-0.6);
    this.handoff.shooterHandoffMotor.set(1);
    this.intake.intakeMotor.set(-1);
    
    if (Utils.getCurrentTimeSeconds() - runTime > bumpPeriod + 0.4) {
      this.intake.desiredPosition = this.intake.DEPLOYED;
      this.runTime = Utils.getCurrentTimeSeconds();
    }
    else if (Utils.getCurrentTimeSeconds() - runTime > bumpPeriod) {
      this.intake.desiredPosition = this.intake.STOWED;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.spindex.spindexMotor.set(0);
    this.handoff.shooterHandoffMotor.set(0);
    this.intake.desiredPosition = this.intake.DEPLOYED;
    this.intake.intakeMotor.set(0);
    PositionData.speedFactor = 1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}