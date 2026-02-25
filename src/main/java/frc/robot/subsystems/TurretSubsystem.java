// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  // All motors part of the subsystem
  public final TalonFX neckMotor = new TalonFX(TurretConstants.neckMotorID, "Aux Can");  // controls horizontal angle of turret
  public final TalonFX anglerMotor = new TalonFX(TurretConstants.anglerMotorID, "Aux Can");
  public final TalonFX shooterLeaderMotor = new TalonFX(TurretConstants.shooterLeadID, "Aux Can");
  public final TalonFX shooterFollowerMotor = new TalonFX(TurretConstants.shooterFollowerID, "Aux Can");
  public final TalonFX shooterHandoffMotor = new TalonFX(TurretConstants.shooterHandoffID, "Aux Can");
  



  // private final StatusSignal<Angle> neckPositionSignal = neckMotor.getPosition();

  final DoublePublisher rotateInfo;
  

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.rotateInfo = table.getDoubleTopic("turret/rotate-angle").publish();
    // configure the shooterFollowerMotor to follow the leader

    TalonFXConfiguration neckConfig = new TalonFXConfiguration();
    neckConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(TurretConstants.neckCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    neckMotor.getConfigurator().apply(neckConfig);

    TalonFXConfiguration anglerConfig = new TalonFXConfiguration();
    anglerConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(TurretConstants.anglerCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    anglerMotor.getConfigurator().apply(anglerConfig);

    TalonFXConfiguration shooterLeadConfig = new TalonFXConfiguration();
    shooterLeadConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(TurretConstants.neckCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

      shooterLeaderMotor.getConfigurator().apply(shooterLeadConfig);

    TalonFXConfiguration shooterFollowConfig = new TalonFXConfiguration();
    shooterFollowConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(TurretConstants.neckCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

      shooterFollowerMotor.getConfigurator().apply(shooterFollowConfig);
      shooterFollowerMotor.setControl(new Follower(TurretConstants.shooterLeadID, MotorAlignmentValue.Opposed));

      TalonFXConfiguration handoffConfig = new TalonFXConfiguration();
    handoffConfig
      .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(TurretConstants.handoffCurrentLimit)
          .withSupplyCurrentLimitEnable(true)
      );

    shooterHandoffMotor.getConfigurator().apply(handoffConfig);
  }

  public double getNeckPosition() {
    return neckMotor.getRotorPosition().refresh().getValueAsDouble() / TurretConstants.neckGearRatio;
  }


  public void driveMotor(Voltage volts) {
    shooterLeaderMotor.setVoltage(volts.in(Volts));
}

public void logMotor(SysIdRoutineLog log) {
  log.motor("shoooter-motor")
    .voltage(shooterLeaderMotor.getMotorVoltage().getValue())
    .angularPosition(shooterLeaderMotor.getPosition().getValue())
    .angularVelocity(shooterLeaderMotor.getVelocity().getValue());
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
    this.rotateInfo.set(getNeckPosition());

    

        // This method will be called once per scheduler run

  }
}
