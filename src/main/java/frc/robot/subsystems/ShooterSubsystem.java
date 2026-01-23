package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFXS rawShooterLeader = new TalonFXS(ShooterConstants.leaderShooterMotorID);
    private final TalonFXS rawShooterFollower = new TalonFXS(ShooterConstants.followerShooterMotorID);
}