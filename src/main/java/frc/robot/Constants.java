package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.controller.*;

public final class Constants {
    public static class PivotConstants {
        public static final int PivotMotorID = 1;
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
        public static final double intakeCurrentLimit = 120;
    }

    public static class HandoffConstants {
        public static final int HandoffMotorID = 16;
    }
}
