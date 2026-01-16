package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.controller.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public final class Constants {
    public static class PivotConstants {
        public static final int PivotMotorID = 1;

        public static final double kPivotP = 50;
        public static final double kPivotI = 0;
        public static final double kPivotD = 0;
        public static final AngularVelocity pivotDegreesPerSecond = DegreesPerSecond.of(90);
        public static final AngularAcceleration pivotDegreesPerSecondPerSecond = DegreesPerSecondPerSecond.of(45);

        public static final Current pivotAmpsLimit = Amps.of(40);
        public static final Time pivotClosedRampRate = Seconds.of(0.25);
        public static final Time pivotOpenRampRate = Seconds.of(0.25);

        public static final MechanismGearing pivotGearbox = new MechanismGearing(GearBox.fromReductionStages(3, 4));

        public static final Angle pivotStartingPosition = Degrees.of(0);
        public static final Angle pivotWrappingPosition = Degrees.of(360);
        public static final Angle pivotHardLimitPosition = Degrees.of(720);

        public static final Distance pivotDistance = Meters.of(0.5);
        public static final Mass pivotMass = Pounds.of(5);
    }

    public static class AnglerConstants {
        public static final int AnglerMotorID = 2;

        public static final double kAnglerP = 4;
        public static final double kAnglerI = 0;
        public static final double kAnglerD = 0;
        public static final AngularVelocity kAnglerDegreesPerSecond = DegreesPerSecond.of(180);
        public static final AngularAcceleration kAnglerDegreesPerSecondPerSecond = DegreesPerSecondPerSecond.of(90);

        public static final ArmFeedforward anglerFeedforward = new ArmFeedforward(0, 0, 0, 0);

        public static final Angle anglerLowSoftLimit = Degrees.of(-30);
        public static final Angle anglerHighSoftLimit = Degrees.of(100);

        public static final MechanismGearing anglerGearbox = new MechanismGearing(GearBox.fromReductionStages(3, 4));

        public static final Current anglerAmpsLimit = Amps.of(40);
        public static final Time anglerClosedRampRate = Seconds.of(0.2);
        public static final Time anglerOpenRampRate = Seconds.of(0.2);

        public static final Angle anglerStartingPosition = Degrees.of(0);
        public static final Angle anglerLowHardLimitPosition = Degrees.of(0);
        public static final Angle anglerHighHardLimitPosition = Degrees.of(180);

        public static final Distance anglerDistance = Meters.of(1);
        public static final Mass anglerMass = Pounds.of(5);
    }

    public static class ShooterConstants {
        public static final int leaderShooterMotorID = 3;
        public static final int followerShooterMotorID = 4;

        public static final double kShooterP = 50;
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;
        public static final AngularVelocity ShooterDegreesPerSecond = DegreesPerSecond.of(90);
        public static final AngularAcceleration ShooterDegreesPerSecondPerSecond = DegreesPerSecondPerSecond.of(45);

        public static final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    
        public static final MechanismGearing shooterGearbox = new MechanismGearing(GearBox.fromReductionStages(3, 4));

        public static final Current shooterAmpsLimit = Amps.of(40);
        public static final Time shooterClosedRampRate = Seconds.of(0.25);
        public static final Time shooterOpenRampRate = Seconds.of(0.25);

        public static final Distance shooterWheelDiameter = Inches.of(4);
        public static final Mass shooterWheelMass = Pounds.of(1);
        public static final AngularVelocity shooterMaxRPM = RPM.of(1000);
    }
}
