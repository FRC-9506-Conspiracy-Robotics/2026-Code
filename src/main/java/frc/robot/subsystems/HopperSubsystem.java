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
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  public final TalonFX hopperMotor = new TalonFX(Constants.HopperConstants.hopperMotorID);
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    hopperConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(HopperConstants.hopperCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    hopperMotor.getConfigurator().apply(hopperConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
