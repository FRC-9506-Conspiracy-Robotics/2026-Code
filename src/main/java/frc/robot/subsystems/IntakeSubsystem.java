package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DeployIntake;

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
  //public final SparkMax deployLeaderMotor = new SparkMax(IntakeConstants.deployLeaderID, MotorType.kBrushless);
  public final TalonFX deployLeaderMotor = new TalonFX(IntakeConstants.deployLeaderID, "rio");
  private final TalonFX deployFollowerMotor = new TalonFX(IntakeConstants.deployFollowerID, "rio");
  //private final SparkMax deployFollowerMotor = new SparkMax(IntakeConstants.deployFollowerID, MotorType.kBrushless);
  //public final RelativeEncoder deployEncoder = deployLeaderMotor.getEncoder();
  //public final RelativeEncoder followerEncoder = deployFollowerMotor.getEncoder();

  public static boolean stopReloading = false;
  public static boolean outOfZone = false;

  final DoublePublisher deployInfo;
  final BooleanPublisher isReloading;
  final BooleanPublisher isDeployed;

  public boolean desiredPosition = false;
  public boolean DEPLOYED = true;
  public boolean STOWED =  false;
  public double deploySpeed = 0.25;
  public boolean unjamming = false;
  public static boolean deploying = false;

  public IntakeSubsystem() {
    //intake motors
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
      //intake deploy motors
      TalonFXConfiguration deployConfig = new TalonFXConfiguration();
      deployConfig
        .withMotorOutput(
          new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(IntakeConstants.deployCurrentLimit)
          .withStatorCurrentLimitEnable(true));

    deployLeaderMotor.getConfigurator().apply(deployConfig);
    deployFollowerMotor.getConfigurator().apply(deployConfig);
    deployFollowerMotor.setControl(new Follower(IntakeConstants.deployLeaderID, MotorAlignmentValue.Opposed));

    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeFollowerMotor.getConfigurator().apply(intakeConfig);
    intakeFollowerMotor.setControl(new Follower(IntakeConstants.intakeID, MotorAlignmentValue.Opposed));

    /*
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
    */

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.deployInfo = table.getDoubleTopic("encoder-info/deploy-motor").publish();
    this.isReloading = table.getBooleanTopic("status/reload-stopped").publish();
    this.isDeployed = table.getBooleanTopic("status/deployed").publish();
  }

  public Command deployIntake() { // Deploys AND retracts intake
    return new DeployIntake(this);
  }

  public Command startIntakeCommand() {
    return startEnd(
      () -> intakeMotor.set(-1),
      () -> intakeMotor.set(0)
      );
  }
  
  public Command unjamIntake() {
    return startEnd(
      () -> {intakeMotor.set(1);
            this.unjamming = true;},
      () -> {intakeMotor.set(0);
            this.unjamming = false;}
      );
  }

  public Command toggleReload() {
    return runOnce(
      () -> IntakeSubsystem.stopReloading = !IntakeSubsystem.stopReloading
    );
  }

  public Command toggleDeploy() {
    return runOnce(
      () -> {this.desiredPosition = !this.desiredPosition;
            this.deploySpeed = 0.35;
            this.unjamming = false;}

    );
  }

  

  @Override
  public void periodic() {
    this.deployInfo.set(deployFollowerMotor.getPosition().getValueAsDouble());
    this.isReloading.set(IntakeSubsystem.stopReloading);
    this.isDeployed.set(this.desiredPosition);

    if (desiredPosition == DEPLOYED && this.deployFollowerMotor.getPosition().getValueAsDouble() > -9) {
      deployLeaderMotor.set(-this.deploySpeed);
      intakeMotor.set(0);
    }
    else if (desiredPosition == STOWED && this.deployFollowerMotor.getPosition().getValueAsDouble() < -1) {
      deployLeaderMotor.set(this.deploySpeed);
      intakeMotor.set(0);
    }
    else if (desiredPosition == DEPLOYED && !this.unjamming && !IntakeSubsystem.stopReloading) {
      deployLeaderMotor.set(0);
      intakeMotor.set(-1);
    }
    else if (!this.unjamming) {
      deployLeaderMotor.set(0);
      intakeMotor.set(0);
    }
    else {
      deployLeaderMotor.set(0);
    }

  }
}