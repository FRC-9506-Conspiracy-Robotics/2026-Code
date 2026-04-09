// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrumShooterConstants;

public class DrumShooterSubsystem extends SubsystemBase {

  // Motor configuration
  public final TalonFX shooterMotor0 = new TalonFX(
    DrumShooterConstants.shooterMotor0ID, "Aux Can");
  public final TalonFX shooterMotor1 = new TalonFX(
    DrumShooterConstants.shooterMotor1ID, "Aux Can");
  public final TalonFX shooterMotor2 = new TalonFX(
    DrumShooterConstants.shooterMotor2ID, "Aux Can");
  public final TalonFX shooterMotor3 = new TalonFX(
    DrumShooterConstants.shooterMotor3ID, "Aux Can");
  public final TalonFX handoffMotor0 = new TalonFX(
    DrumShooterConstants.handoffMotor0, "Aux Can");
  public final TalonFX handoffMotor1 = new TalonFX(
    DrumShooterConstants.handoffMotor1, "Aux Can");

  /** Creates a new DrumShooter. */
  public DrumShooterSubsystem() {}

  public Command lockOn() {
    return startEnd(
      () -> {RobotContainer.lockOn = true;
            PositionData.accumulatedError = 0;}, 
      () -> RobotContainer.lockOn = false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
