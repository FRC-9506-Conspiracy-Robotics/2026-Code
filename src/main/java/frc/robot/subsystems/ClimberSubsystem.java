// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private boolean rising = false;
  private boolean falling = false;

  public final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID, "rio");
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ClimberConstants.climberCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    climberMotor.getConfigurator().apply(climberConfig);
  }

  public Command raiseClimber() {
    return startEnd(
      () -> this.rising = true, 
      () -> this.rising = false
      );
  }
  public Command lowerClimber() {
    return startEnd(
      () -> this.falling = true,
      () -> this.falling = false
      );
  }

  @Override
  public void periodic() {
    if (climberMotor.getPosition().refresh().getValueAsDouble() > -110 && rising) {
      climberMotor.set(-0.2);
    }
    else if (climberMotor.getPosition().refresh().getValueAsDouble() < -5 && falling) {
      climberMotor.set(0.5);
    }
    else {
      climberMotor.set(0);
    }
    // This method will be called once per scheduler run
  }
}
