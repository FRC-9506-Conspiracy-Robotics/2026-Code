package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // someone learn how to make followers this code is so bad

    private final TalonFXS rawShooterMotor1 = new TalonFXS(ShooterConstants.leaderShooterMotorID);
    private final SmartMotorControllerConfig shooterMotorConfig1 = new SmartMotorControllerConfig()
        .withTelemetry("rawShooter1", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withClosedLoopController(
            ShooterConstants.kShooterP, 
            ShooterConstants.kShooterI, 
            ShooterConstants.kShooterD, 
            ShooterConstants.ShooterDegreesPerSecond, 
            ShooterConstants.ShooterDegreesPerSecondPerSecond
        )
        .withFeedforward(ShooterConstants.shooterFeedforward)
        .withGearing(ShooterConstants.shooterGearbox)
        .withStatorCurrentLimit(ShooterConstants.shooterAmpsLimit)
        .withClosedLoopRampRate(ShooterConstants.shooterClosedRampRate)
        .withOpenLoopRampRate(ShooterConstants.shooterOpenRampRate);

    private final TalonFXS rawShooterMotor2 = new TalonFXS(ShooterConstants.followerShooterMotorID);
    private final SmartMotorControllerConfig shooterMotorConfig2 = shooterMotorConfig1
        .withTelemetry("rawShooter2", TelemetryVerbosity.HIGH)
        .withMotorInverted(true);

    private final SmartMotorController motor1 = new TalonFXSWrapper(rawShooterMotor1, DCMotor.getKrakenX60(1), shooterMotorConfig1);
    private final FlyWheelConfig flyWheelConfig1 = new FlyWheelConfig(motor1)
        .withTelemetry("flyWheel1", TelemetryVerbosity.HIGH)
        .withDiameter(ShooterConstants.shooterWheelDiameter)
        .withMass(ShooterConstants.shooterWheelMass)
        .withUpperSoftLimit(ShooterConstants.shooterMaxRPM);

    private final SmartMotorController motor2 = new TalonFXSWrapper(rawShooterMotor2, DCMotor.getKrakenX60(1), shooterMotorConfig2);
    private final FlyWheelConfig flyWheelConfig2 = new FlyWheelConfig(motor2)
        .withTelemetry("flyWheel2", TelemetryVerbosity.HIGH)
        .withDiameter(ShooterConstants.shooterWheelDiameter)
        .withMass(ShooterConstants.shooterWheelMass)
        .withUpperSoftLimit(ShooterConstants.shooterMaxRPM);

    private final FlyWheel shooterMotor1 = new FlyWheel(flyWheelConfig1);
    private final FlyWheel shooterMotor2 = new FlyWheel(flyWheelConfig2);

    public ShooterSubsystem() {}

    public Command setShooterRPMCommand(double rpm) {
        return runOnce(() -> {
            shooterMotor1.setSpeed(RPM.of(rpm));
            shooterMotor2.setSpeed(RPM.of(rpm));
        });
    }

    public Command stopShooterCommand() {
        return runOnce(() -> {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        });
    }
}