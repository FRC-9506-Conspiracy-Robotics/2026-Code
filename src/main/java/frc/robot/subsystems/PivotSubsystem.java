package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFXS rawPivotMotor = new TalonFXS(PivotConstants.PivotMotorID);
    private final SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig(this)
        .withTelemetry("rawPivotMotor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withClosedLoopController(
            PivotConstants.kPivotP, 
            PivotConstants.kPivotI, 
            PivotConstants.kPivotD, 
            PivotConstants.pivotDegreesPerSecond, 
            PivotConstants.pivotDegreesPerSecondPerSecond
        )
        .withStatorCurrentLimit(PivotConstants.pivotAmpsLimit)
        .withClosedLoopRampRate(PivotConstants.pivotClosedRampRate)
        .withOpenLoopRampRate(PivotConstants.pivotOpenRampRate)
        .withGearing(PivotConstants.pivotGearbox);

    private final SmartMotorController motor = new TalonFXSWrapper(rawPivotMotor, DCMotor.getFalcon500(1), pivotMotorConfig);
    private final PivotConfig pivotConfig = new PivotConfig(motor)
        .withTelemetry("pivotMotor", TelemetryVerbosity.HIGH)
        .withStartingPosition(PivotConstants.pivotStartingPosition)
        .withWrapping(PivotConstants.pivotStartingPosition, PivotConstants.pivotWrappingPosition)
        .withHardLimit(PivotConstants.pivotStartingPosition, PivotConstants.pivotWrappingPosition)
        .withMOI(PivotConstants.pivotDistance, PivotConstants.pivotMass);

    private final Pivot pivotMotor = new Pivot(pivotConfig);

    public PivotSubsystem() {}

    public Command setPivotPositionCommand(Angle positionDegrees) {
        return runOnce(() -> pivotMotor.setAngle(positionDegrees));
    }

    public Command stopPivotCommand() {
        return runOnce(() -> pivotMotor.set(0));
    }
}