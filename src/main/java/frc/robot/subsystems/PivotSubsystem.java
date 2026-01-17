package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFXS rawPivotMotor = new TalonFXS(PivotConstants.PivotMotorID);
}