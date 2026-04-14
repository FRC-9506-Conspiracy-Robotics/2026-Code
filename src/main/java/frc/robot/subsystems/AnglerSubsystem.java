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
import frc.robot.Constants.AnglerConstants;
import frc.robot.subsystems.PositionData.Pose;

public class AnglerSubsystem extends SubsystemBase {

  public static boolean outOfAllianceZone = false;
  public static boolean farFromHub = false;
  public static boolean trenchSafety = false;

  public final TalonFX anglerMotor = new TalonFX(
    AnglerConstants.anglerMotorID, "Aux Can");
  
  private PositionData positionData;

  /** Creates a new AnglerSubsystem. */
  public AnglerSubsystem(PositionData positionData_) {
    this.positionData = positionData_;

    TalonFXConfiguration anglerConfig = new TalonFXConfiguration();
    anglerConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(AnglerConstants.anglerCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );

    anglerMotor.getConfigurator().apply(anglerConfig);
  }

  @Override
  public void periodic() {
    double kP = 0.1;
    double signal = 0;
    Pose pose = positionData.getPose();
    if (pose.x > 3.6 - (4.6 * Math.abs(pose.velX) * 0.15) && pose.x < 5.3 + (4.6 * Math.abs(pose.velX) * 0.15) || pose.x > 11 - (12 * Math.abs(pose.velX) * 0.15) && pose.x < 13 + (12 * Math.abs(pose.velX) * 0.15)) {
      trenchSafety = true;
    }
    else {
      trenchSafety = false;
    }
    if (pose.x > 5.5) {
      outOfAllianceZone = true;
    }
    else if (pose.x < 5.2) {
      outOfAllianceZone = false;
    }
    if (positionData.getDistance() > 3.5) {
      farFromHub = true;
    }
    else if (positionData.getDistance() < 3.3) {
      farFromHub = false;
    }
    if (trenchSafety) {
      signal = (0 - anglerMotor.getPosition().refresh().getValueAsDouble() * kP);
    }
    else if (outOfAllianceZone) {
      signal = (6 - anglerMotor.getPosition().refresh().getValueAsDouble()) * kP;
    }
    else if (farFromHub) {
      signal = (3 - anglerMotor.getPosition().refresh().getValueAsDouble()) * kP;
    }
    else {
      signal = (0 - anglerMotor.getPosition().refresh().getValueAsDouble()) * kP;
    }
    if (signal > 1) {
      signal = 1;
    }
    else if (signal < -1) {
      signal = -1;
    }
    anglerMotor.set(signal);
    // This method will be called once per scheduler run
  }
}
