package frc.robot.subsystems;

import edu.wpi.first.math.Num;
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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID);
  private final SparkMax deployLeaderMotor = new SparkMax(IntakeConstants.deployLeaderID, MotorType.kBrushless);
  private final SparkMax deployFollowerMotor = new SparkMax(IntakeConstants.deployFollowerID, MotorType.kBrushless);
  private final RelativeEncoder deployEncoder = deployLeaderMotor.getEncoder();
  private final double deployVelocity = deployEncoder.getVelocity();

  final DoublePublisher deployInfo;

  public static boolean deployed = false;
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

  public void deployIntake() { // Deploys AND retracts intake
    if ((deploying == false) && (deployed == false)) { // Runs if not deploying or retracting intake and the intake is not deployed yet
      deploying = true;
      double adjustedSpeed = 0.00; // Will increase to desired speed based on position
      while (deployed == false) { // Runs until we are fully extended
        if (deployVelocity > (-21 - deployEncoder.getPosition())) { //
          adjustedSpeed += -0.01;
          deployLeaderMotor.set(adjustedSpeed); // Adjusts speed if it is below target velocity
        }
        else if (deployVelocity < (-21 - deployEncoder.getPosition())) {
          adjustedSpeed += 0.01;
          deployLeaderMotor.set(adjustedSpeed); // Adjusts speed if it is above target velocity
        }
        if (deployEncoder.getPosition() <= -13) { // Runs when extended
          deployLeaderMotor.set(0);
          deployed = true;
          deploying = false;
          intakeMotor.set(-1);
        }
        try {
          Thread.sleep(40); // Waits 0.04 seconds before repeating the while loop
        } catch (InterruptedException e) {
          deployLeaderMotor.set(0);
          deploying = false;
          System.out.print("Deploying intake was interrupted, stopped deploying.");
          Thread.currentThread().interrupt(); 
        }
      } 
    } else if ((deploying == false) && (deployed == true)) { // Runs if not deploying or retracting intake and if the intake is already deployed
      deploying = true;
      double adjustedSpeed = 0.00; // Increases to desired speed based on position.
      intakeMotor.set(0); // Turns off intake before retracting
      while (deployed == true) { // Runs until we are fully retracted
        if (deployVelocity < (7 - deployEncoder.getPosition())) {
          adjustedSpeed += 0.01;
          deployLeaderMotor.set(adjustedSpeed); // Adjusts speed if it is below target velocity
        }
        else if (deployVelocity > (7 + deployEncoder.getPosition())) {
          adjustedSpeed += -0.01;
          deployLeaderMotor.set(adjustedSpeed); // Adjusts speed if it is above target velocity
        }
        if (deployEncoder.getPosition() >= -1) { // Runs when retracted
          deployLeaderMotor.set(0);
          deployed = false;
          deploying = false;
        }
        try {
          Thread.sleep(40); // Waits 0.04 seconds before repeating the while loop
        } catch (InterruptedException e) {
          deployLeaderMotor.set(0);
          deploying = false;
          System.out.print("Deploying intake was interrupted, stopped deploying.");
          Thread.currentThread().interrupt(); 
        }
      }
    }
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
