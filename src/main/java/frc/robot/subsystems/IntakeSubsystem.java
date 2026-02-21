package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DeployIntake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
  public final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID, "rio");
  public final SparkMax deployLeaderMotor = new SparkMax(IntakeConstants.deployLeaderID, MotorType.kBrushless);
  private final SparkMax deployFollowerMotor = new SparkMax(IntakeConstants.deployFollowerID, MotorType.kBrushless);
  public final RelativeEncoder deployEncoder = deployLeaderMotor.getEncoder();

  final DoublePublisher deployInfo;

  public boolean desiredPosition = false;
  public boolean DEPLOYED = true;
  public boolean STOWED =  false;
  public static boolean deploying = false;

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
          .withSupplyCurrentLimitEnable(true)
      );

    intakeMotor.getConfigurator().apply(intakeConfig);

    SparkMaxConfig deployLeaderConfig = new SparkMaxConfig();
    deployLeaderConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.deployCurrentLimit);

    deployLeaderMotor.configure(deployLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig deployFollowerConfig = new SparkMaxConfig();
    deployFollowerConfig
      .idleMode(IdleMode.kBrake)
      .follow(deployLeaderMotor, true);

    deployFollowerMotor.configure(deployFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    this.deployInfo = table.getDoubleTopic("encoder-info/deploy-motor").publish();
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

  @Override
  public void periodic() {
    this.deployInfo.set(deployEncoder.getPosition());
  }
}
