// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.TurretConstants;



// public class HandoffSubsystem extends SubsystemBase {
//     public final TalonFX shooterHandoffMotor = new TalonFX(TurretConstants.shooterHandoffID, "Aux Can");

//   /** Creates a new HandoffSubsystem. */
//   public HandoffSubsystem() {

//     TalonFXConfiguration handoffConfig = new TalonFXConfiguration();
//     handoffConfig = handoffConfig.withMotorOutput(
//       new MotorOutputConfigs()
//       .withNeutralMode(NeutralModeValue.Brake)
//     ).withCurrentLimits(
//       new CurrentLimitsConfigs()
//       .withStatorCurrentLimit(TurretConstants.handoffCurrentLimit)
//       .withStatorCurrentLimitEnable(true)
//     );
//     this.shooterHandoffMotor.getConfigurator().apply(handoffConfig);
//   }

//   public Command unjamShooter() {
//     return startEnd(
//       () -> shooterHandoffMotor.set(-0.75),
//       () -> shooterHandoffMotor.set(0)
//     );
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
