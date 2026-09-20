
package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final Mode realMode = Mode.REAL;
    public static final Mode currentMode = Utils.isSimulation() ? Mode.SIM : realMode;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    
    public static class DriverConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
    }

    public static class SwerveConstants {
        public static final double maxDriveSpeed = Units.feetToMeters(15);
    }

    public static class ShooterConstants {
        public static final int leaderShooterMotorID = 3;
        public static final int followerShooterMotorID = 4;
    }

    // everything below will be for drum shooter

    public static class IntakeConstants {
        public enum IntakeState {
            DEPLOYED,
            STOWED
        };
        
        public static final int deployLeaderID = 13;
        public static final int deployFollowerID = 14;
        public static final int intakeID = 15;
        public static final int intakeFollowerID = 23;
        
        public static final double intakeSpeed = -0.85;

        public static final int deployGearRatio = 15;
        public static final int intakeGearRatio = 2;

        public static final int deployCurrentLimit = 40;
        public static final int intakeCurrentLimit = 25;

        public static final double deployPosition = -8.5;
        public static final double stowedPosition = -0.5;
        public static final double deployTolerance = 0.5;
    }

    public static class DrumShooterConstants {
        public static final int shooterMotorLeadTL = 17;
        public static final int shooterMotorFollowerBL = 18;
        public static final int shooterMotorFollowerTR = 19;
        public static final int shooterMotorFollowerBR = 20;

        public static final int centerHandoffMotor = 21;
        public static final int rampHandoffMotor = 22;

        public static final int shooterCurrentLimit = 40;
        public static final int centerHandoffCurrentLimit = 25;
        public static final int rampHandoffCurrentLimit = 30;

        public static final double kS = 0.5566;
        public static final double kV = 0.28884;
    }

    public static class HopperConstants {
        public static final int hopperMotorID = 16;

        public static final int hopperCurrentLimit = 30;
    }

    public static class AnglerConstants {
        public static final int anglerMotorID = 24;

        public static final int anglerCurrentLimit = 25;
    }

    public static class LimelightNames {
        public static final String limelight4AFront = "limelight-afouri";
        public static final String limelight3ALeft = "limelight-athree";
        public static final String limelight3ARight = "limelight-athreei";
    }
    
}