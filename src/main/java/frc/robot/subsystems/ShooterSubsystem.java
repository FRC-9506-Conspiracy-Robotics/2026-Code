package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.Pair;
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
    private final TalonFXS rawShooterLeader = new TalonFXS(ShooterConstants.leaderShooterMotorID);
    private final TalonFXS rawShooterFollower = new TalonFXS(ShooterConstants.followerShooterMotorID);
    private final SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig(this)
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
        .withOpenLoopRampRate(ShooterConstants.shooterOpenRampRate)
        .withFollowers(Pair.of(rawShooterFollower, true));
        // this might be .withFollowers(Pair.of(ShooterConstants.followerShooterMotorID, true or false)); the docs for following suck

    private final SmartMotorController motor = new TalonFXSWrapper(rawShooterLeader, DCMotor.getKrakenX60(1), shooterMotorConfig);
    private final FlyWheelConfig flyWheelConfig = new FlyWheelConfig(motor)
        .withTelemetry("flyWheel1", TelemetryVerbosity.HIGH)
        .withDiameter(ShooterConstants.shooterWheelDiameter)
        .withMass(ShooterConstants.shooterWheelMass)
        .withUpperSoftLimit(ShooterConstants.shooterMaxRPM);

    private final FlyWheel shooterMotorLeader = new FlyWheel(flyWheelConfig);

    public ShooterSubsystem() {}

    public Command setShooterRPMCommand(double rpm) {
        return runOnce(() -> {
            shooterMotorLeader.setSpeed(RPM.of(rpm));
        });
    }

    public Command stopShooterCommand() {
        return runOnce(() -> {
            shooterMotorLeader.set(0);
        });
    }
}