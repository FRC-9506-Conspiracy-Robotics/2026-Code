// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrumShooterConstants;
import frc.robot.Constants.HopperConstants;

public class DrumShooterSubsystem extends SubsystemBase {

  private double shooterTuning = 1;
  private double centerSpeed = -0.45;
  private double rampSpeed = 0.75;
  private double hopperSpeed = 0.75;
  private double handoffDelay = 1.5;

  public boolean unjamming = false;
  public boolean loading = false;
  public static boolean shooting = false;
  public double timer = 0;
  

  // Motor configuration
  public final TalonFX shooterMotorLeadTL = new TalonFX(
    DrumShooterConstants.shooterMotorLeadTL, "Aux Can");
  public final TalonFX shooterMotorFollowerBL = new TalonFX(
    DrumShooterConstants.shooterMotorFollowerBL, "Aux Can");
  public final TalonFX shooterMotorFollowerTR = new TalonFX(
    DrumShooterConstants.shooterMotorFollowerTR, "Aux Can");
  public final TalonFX shooterMotorFollowerBR = new TalonFX(
    DrumShooterConstants.shooterMotorFollowerBR, "Aux Can");

  public final TalonFX centerHandoffMotor = new TalonFX(
    DrumShooterConstants.centerHandoffMotor, "Aux Can");
  public final TalonFX rampHandoffMotor = new TalonFX(
    DrumShooterConstants.rampHandoffMotor, "Aux Can");

  public final TalonFX hopperMotor = new TalonFX(
    HopperConstants.hopperMotorID, "rio");

  private PositionData positionData;

  /** Creates a new DrumShooter. */
  public DrumShooterSubsystem(PositionData positionData_) {
    this.positionData = positionData_;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(DrumShooterConstants.shooterCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );

    shooterMotorLeadTL.getConfigurator().apply(shooterConfig);

    shooterMotorFollowerBL.getConfigurator().apply(shooterConfig);
    shooterMotorFollowerBL.setControl(new Follower(DrumShooterConstants.shooterMotorLeadTL, MotorAlignmentValue.Aligned));

    shooterMotorFollowerTR.getConfigurator().apply(shooterConfig);
    shooterMotorFollowerTR.setControl(new Follower(DrumShooterConstants.shooterMotorLeadTL, MotorAlignmentValue.Opposed));

    shooterMotorFollowerBR.getConfigurator().apply(shooterConfig);
    shooterMotorFollowerBR.setControl(new Follower(DrumShooterConstants.shooterMotorLeadTL, MotorAlignmentValue.Opposed));

    TalonFXConfiguration centerHandoffConfig = new TalonFXConfiguration();
    centerHandoffConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(DrumShooterConstants.centerHandoffCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );

    centerHandoffMotor.getConfigurator().apply(centerHandoffConfig);

    TalonFXConfiguration rampHandoffConfig = new TalonFXConfiguration();
    rampHandoffConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(DrumShooterConstants.rampHandoffCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );

    rampHandoffMotor.getConfigurator().apply(rampHandoffConfig);

    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    hopperConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(HopperConstants.hopperCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );

    hopperMotor.getConfigurator().apply(hopperConfig);

  }

  public Command lockOn() {
    return startEnd(
      () -> {RobotContainer.lockOn = true;
            PositionData.accumulatedError = 0;}, 
      () -> RobotContainer.lockOn = false);
  }

  public Command unjamShooter() {
    return startEnd(
      () -> this.unjamming = true,
      () -> this.unjamming = false);
  }

  public Command toggleShooter() {
    return runOnce( () -> DrumShooterSubsystem.shooting = !DrumShooterSubsystem.shooting );
  }

  public void driveMotor(Voltage volts) {
    shooterMotorLeadTL.setVoltage(volts.in(Volts));
  }



  public void logMotor(SysIdRoutineLog log) {
    log.motor("shoooter-motor")
      .voltage(shooterMotorLeadTL.getMotorVoltage().getValue())
      .angularPosition(shooterMotorLeadTL.getPosition().getValue())
      .angularVelocity(shooterMotorLeadTL.getVelocity().getValue());
  }

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              this::driveMotor,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              this::logMotor,
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    double angle = 80 * Math.PI / 180;
    if (AnglerSubsystem.outOfAllianceZone) {
      angle = 45 * Math.PI / 180; // Angle of hood outside of alliance zone
    }
    else if (AnglerSubsystem.farFromHub) {
      angle = 62.5 * Math.PI / 180;
    }
    double vel = this.positionData.getVelocity(angle, this.positionData.getDistance());

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrumShooterConstants.kS, DrumShooterConstants.kV);
    double feedforwardSignal = feedforward.calculate(vel * shooterTuning);

    double volts = -feedforwardSignal;

    if (volts > 12) {
      volts = 6;
    }
    else if (volts < -12) {
      volts = -6;
    }

    if (unjamming == true) {
      volts = 3;
    }
    else if (shooting == false) {
      volts = 0;
    }

    double distanceCorrection = 1;
    if (positionData.getDistance() > 2.75) {
      distanceCorrection = 1 + ((positionData.getDistance() - 2.75) * 0.15);
    }
    if (distanceCorrection > 1.5) {
      distanceCorrection = 1.5;
    }

    double angleCorrection = 1;
    if (AnglerSubsystem.farFromHub) {
      angleCorrection = 0.85;
    }

    this.shooterMotorLeadTL.setVoltage(volts * distanceCorrection);

    if (unjamming) {
      this.rampHandoffMotor.set(-rampSpeed);
      this.centerHandoffMotor.set(-centerSpeed);
      this.hopperMotor.set(-hopperSpeed);
    }
    else if (loading && timer + handoffDelay < Utils.getCurrentTimeSeconds()) {
      this.rampHandoffMotor.set(rampSpeed);
      this.centerHandoffMotor.set(centerSpeed);
      this.hopperMotor.set(hopperSpeed);
    }
    else {
      this.rampHandoffMotor.set(0);
      this.centerHandoffMotor.set(0);
      this.hopperMotor.set(0);
    }

    // This method will be called once per scheduler run
  }
}
