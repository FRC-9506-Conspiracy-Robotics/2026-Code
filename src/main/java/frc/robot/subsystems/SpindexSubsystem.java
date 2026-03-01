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
import frc.robot.Constants;
import frc.robot.Constants.SpindexConstants;

public class SpindexSubsystem extends SubsystemBase {
  public final TalonFX spindexMotor = new TalonFX(Constants.SpindexConstants.spindexMotorID);
  /** Creates a new spindexSubsystem. */
  public SpindexSubsystem() {
    TalonFXConfiguration spindexConfig = new TalonFXConfiguration();
    spindexConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(SpindexConstants.spindexCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    spindexMotor.getConfigurator().apply(spindexConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}