package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.AnglerConstants;

public class AnglerSubsytem extends SubsystemBase { 
    private final TalonFXS rawAnglerMotor = new TalonFXS(AnglerConstants.AnglerMotorID);
    private final SmartMotorControllerConfig anglerMotorConfig = new SmartMotorControllerConfig(this)
        .withTelemetry("rawArmMotor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)  
        .withIdleMode(MotorMode.BRAKE)
        .withClosedLoopController(
            AnglerConstants.kAnglerP, 
            AnglerConstants.kAnglerI, 
            AnglerConstants.kAnglerD, 
            AnglerConstants.kAnglerDegreesPerSecond,
            AnglerConstants.kAnglerDegreesPerSecondPerSecond
        )
        .withFeedforward(AnglerConstants.anglerFeedforward)
        .withSoftLimit(AnglerConstants.anglerLowSoftLimit, AnglerConstants.anglerHighSoftLimit)
        .withGearing(AnglerConstants.anglerGearbox)
        .withStatorCurrentLimit(AnglerConstants.anglerAmpsLimit)
        .withClosedLoopRampRate(AnglerConstants.anglerClosedRampRate)
        .withOpenLoopRampRate(AnglerConstants.anglerOpenRampRate);
    
    // could have a MechanismPositionConfig() added if wanted for predetermided states
    
    private final SmartMotorController motor = new TalonFXSWrapper(rawAnglerMotor, DCMotor.getFalcon500(1), anglerMotorConfig);
    private final ArmConfig armConfig = new ArmConfig(motor)
        .withTelemetry("armMotor", TelemetryVerbosity.HIGH)
        .withStartingPosition(AnglerConstants.anglerStartingPosition)
        .withLength(AnglerConstants.anglerDistance)
        .withMass(AnglerConstants.anglerMass);

    private final Arm anglerMotor = new Arm(armConfig);

    public AnglerSubsytem() {}

    public Command setAnglerPositionCommand(Angle angle) {
        return runOnce(() -> anglerMotor.setAngle(angle));
    }

    public Command stopAnglerCommand() {
        return runOnce(() -> anglerMotor.set(0));
    }
}