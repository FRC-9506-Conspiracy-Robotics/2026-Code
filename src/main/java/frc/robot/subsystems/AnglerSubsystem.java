// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;

public class AnglerSubsystem extends SubsystemBase {

  public final TalonFX anglerMotor = new TalonFX(
    AnglerConstants.anglerMotorID, "Aux Can");

  /** Creates a new AnglerSubsystem. */
  public AnglerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
