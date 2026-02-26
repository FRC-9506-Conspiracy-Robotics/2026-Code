package frc.robot;



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

        public static final int deployCurrentLimit = 40;
        public static final int intakeCurrentLimit = 10;
    }

    public static class TurretConstants {
        public static final int neckMotorID = 17; 
        public static final int anglerMotorID = 18;
        public static final int shooterLeadID = 19;
        public static final int shooterFollowerID = 20;
        public static final int shooterHandoffID = 21;

        public static final double neckGearRatio = 37.5;
        public static final double anglerGearRatio = 66.6667;

        public static final int neckCurrentLimit = 40;
        public static final int anglerCurrentLimit = 20;
        public static final int handoffCurrentLimit = 40;

        public static final double circumferenceOfWheel = 0.319186;

        public static final double shooterkV = 0.12079;
        public static final double shooterkA = 0;
        public static final double shooterkS = 0.24189;
    }

    public static class HopperConstants {
        public static final int hopperMotorID = 16;

        public static final int hopperCurrentLimit = 20;
    }
}
