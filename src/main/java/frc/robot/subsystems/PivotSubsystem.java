package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(PivotConstants.PivotMotorID);

    public PivotSubsystem() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(PivotConstants.pivotCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
            );
        
        pivotMotor.getConfigurator().apply(pivotConfig);
    }
}