package frc.robot.subsystems;

import java.security.PrivateKey;

import org.photonvision.PhotonTargetSortMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.AnglerConstants;

public class AnglerSubsytem extends SubsystemBase { 
    private final TalonFXS anglerMotor = new TalonFXS(AnglerConstants.AnglerMotorID);
    private final TalonFXConfigurator anglerConifig = new TalonFXConfigurator(AnglerConstants.AnglerMotorID);

    public AnglerSubsytem() {
        anglerMotor.getConfigurator().apply(anglerConifig);
    }
}