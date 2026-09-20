package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
  public final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID, "rio");
  public final TalonFX intakeFollowerMotor = new TalonFX(IntakeConstants.intakeFollowerID, "rio");
  public final SparkMax deployLeaderMotor = new SparkMax(IntakeConstants.deployLeaderID, MotorType.kBrushless);
  private final SparkMax deployFollowerMotor = new SparkMax(IntakeConstants.deployFollowerID, MotorType.kBrushless);
  public final RelativeEncoder deployEncoder = deployLeaderMotor.getEncoder();
  public final RelativeEncoder followerEncoder = deployFollowerMotor.getEncoder();


  final DoublePublisher deployInfo;
  final BooleanPublisher isReloading;
  final BooleanPublisher isDeployed;

  public IntakeConstants.IntakeState currentState = IntakeConstants.IntakeState.STOWED;
  public IntakeConstants.IntakeState intakeState = IntakeConstants.IntakeState.STOWED;
  public double deploySpeed = 0.25;
  public boolean unjamming = false;
  public static boolean reloading = false;

  public IntakeSubsystem() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(IntakeConstants.intakeCurrentLimit)
          .withStatorCurrentLimitEnable(true)
      );
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeFollowerMotor.getConfigurator().apply(intakeConfig);
    intakeFollowerMotor.setControl(new Follower(IntakeConstants.intakeID, MotorAlignmentValue.Opposed));

    SparkMaxConfig deployLeaderConfig = new SparkMaxConfig();
    deployLeaderConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(IntakeConstants.deployCurrentLimit);
    deployLeaderMotor.configure(deployLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig deployFollowerConfig = new SparkMaxConfig();
    deployFollowerConfig
      .idleMode(IdleMode.kCoast)
      .follow(deployLeaderMotor, true);
    deployFollowerMotor.configure(deployFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.deployInfo = table.getDoubleTopic("encoder-info/deploy-motor").publish();
    this.isReloading = table.getBooleanTopic("status/reload-stopped").publish();
    this.isDeployed = table.getBooleanTopic("status/deployed").publish();
  }

  public Command toggleIntake() {
    return runOnce(
      () -> {
        if (this.currentState == IntakeConstants.IntakeState.DEPLOYED) {
          this.currentState = IntakeConstants.IntakeState.STOWED;
        } else {
          this.currentState = IntakeConstants.IntakeState.DEPLOYED;
        }
        this.deploySpeed = 0.35;
        this.unjamming = false;}
    );
  }

  public Command startIntakeCommand() {
    return startEnd(
      () -> intakeMotor.set(-1),
      () -> intakeMotor.set(0)
    );
  }

  public Command unjamIntake() {
    return startEnd(
      () -> {
        intakeMotor.set(-IntakeConstants.intakeSpeed);
        this.unjamming = true;
      },
      () -> {
        intakeMotor.set(0);
        this.unjamming = false;
      }
    );
  }

  public Command toggleReload() {
    return runOnce(
      () -> IntakeSubsystem.reloading = !IntakeSubsystem.reloading
    );
  }

  @Override
  public void periodic() {
    this.deployInfo.set(followerEncoder.getPosition());
    this.isReloading.set(IntakeSubsystem.reloading);
    this.isDeployed.set(this.intakeState == IntakeConstants.IntakeState.DEPLOYED);

    if (currentState == IntakeConstants.IntakeState.DEPLOYED && this.deployEncoder.getPosition() > IntakeConstants.deployPosition - IntakeConstants.deployTolerance) { // intake deploying not fully deployed
      deployLeaderMotor.set(-this.deploySpeed);
      intakeMotor.set(0);
    }
    else if (currentState == IntakeConstants.IntakeState.STOWED && this.deployEncoder.getPosition() < IntakeConstants.stowedPosition - IntakeConstants.deployTolerance) { // intake stowing not fully stowed
      deployLeaderMotor.set(this.deploySpeed);
      intakeMotor.set(0);
    }
    else if (currentState == IntakeConstants.IntakeState.DEPLOYED && !this.unjamming && !IntakeSubsystem.reloading) { // intake deployed and running
      deployLeaderMotor.set(0);
      intakeMotor.set(IntakeConstants.intakeSpeed);
    }
    else if (!this.unjamming) { // intake unjaming
      deployLeaderMotor.set(0);
      intakeMotor.set(0);
    }
    else { // intake not running
      deployLeaderMotor.set(0);
    }
  }
}