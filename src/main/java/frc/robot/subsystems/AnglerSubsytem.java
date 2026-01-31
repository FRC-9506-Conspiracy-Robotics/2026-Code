package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;

public class AnglerSubsytem extends SubsystemBase { 
    private final TalonFXS anglerMotor = new TalonFXS(AnglerConstants.AnglerMotorID);
    private final TalonFXConfiguration anglerConifig = new TalonFXConfiguration();

    public AnglerSubsytem() {
    }
}