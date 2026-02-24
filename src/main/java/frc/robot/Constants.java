package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.controller.*;

public final class Constants {
    public static class PivotConstants {
        public static final int PivotMotorID = 1;

        public static final int pivotCurrentLimit = 50;
    }

    public static class AnglerConstants {
        public static final int AnglerMotorID = 2;
    }

    public static class ShooterConstants {
        public static final int leaderShooterMotorID = 3;
        public static final int followerShooterMotorID = 4;
    }

    public static class IntakeConstants {
        public static final int deployLeaderID = 13;
        public static final int deployFollowerID = 14;
        public static final int intakeID = 15;

        public static final int deployGearRatio = 15;
        public static final int intakeGearRatio = 2;

        public static final int deployCurrentLimit = 50;
        public static final int intakeCurrentLimit = 20;
    }

    public static class TurretConstants {
        public static final int neckMotorID = 17; 
        public static final int anglerMotorID = 18;
        public static final int shooterLeadID = 19;
        public static final int shooterFollowerID = 20;
        public static final int shooterHandoffID = 21;

        public static final double neckGearRatio = 37.5;

        public static final int neckCurrentLimit = 40;

        public static final double shooterkV = 0.37844;
        public static final double shooterkA = 0.021975;
        public static final double shooterkS = 0.24189;
    }

    public static class HandoffConstants {
        public static final int HandoffMotorID = 16;
    }
}
