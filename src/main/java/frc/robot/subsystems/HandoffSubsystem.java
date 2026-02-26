// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class HandoffSubsystem extends SubsystemBase {
  public final TalonFX shooterHandoffMotor = new TalonFX(TurretConstants.shooterHandoffID, "Aux Can");

  /** Creates a new HandoffSubsystem. */
  public HandoffSubsystem() {

    TalonFXConfiguration handoffConfig = new TalonFXConfiguration();
    handoffConfig = handoffConfig.withMotorOutput(
      new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
    ).withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimit(TurretConstants.handoffCurrentLimit)
      .withSupplyCurrentLimitEnable(true)
    );
    this.shooterHandoffMotor.getConfigurator().apply(handoffConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
