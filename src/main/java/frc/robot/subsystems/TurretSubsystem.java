// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  // All motors part of the subsystem
  public final TalonFX neckMotor = new TalonFX(TurretConstants.neckMotorID);  // controls horizontal angle of turret
  public final TalonFX anglerMotor = new TalonFX(TurretConstants.anglerMotorID);
  public final TalonFX shooterLeaderMotor = new TalonFX(TurretConstants.shooterLeadID);
  public final TalonFX shooterFollowerMotor = new TalonFX(TurretConstants.shooterFollowerID);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    // configure the shooterFollowerMotor to follow the leader
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
