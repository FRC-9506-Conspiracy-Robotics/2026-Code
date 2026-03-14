
package frc.robot;

import static edu.wpi.first.units.Units.Inches;

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

    public static class IntakeConstants {
        public static final int deployLeaderID = 13;
        public static final int deployFollowerID = 14;
        public static final int intakeID = 15;
        public static final int intakeFollowerID = 23;
        

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

    public static class SpindexConstants {
        public static final int spindexMotorID = 16;

        public static final int spindexCurrentLimit = 40;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 22;

        public static final int climberCurrentLimit = 40;

        public static final int climberGearRatio = 36;
    }

    public static class LimelightNames {
        public static final String limelight4AFront = "limelight-afouri";
        public static final String limelight3ALeft = "limelight-athree";
        public static final String limelight3ARight = "limelight-athreei";
    }

    public static class TunerConstants {
        public static final double[] frontLeftPos = {11.375, 10.375};
        public static final double[] backLeftPos = {-11.375, 10.375};
        public static final double[] backRightPos = {-11.375, -10.375};
        public static final double[] frontRightPos = {11.375, -10.375};
        public static final double driveMotorGearRatio = 6.03;
        public static final double angleMotorGearRatio = 26.09;
        public static final double driveWheelDiameter = 4; // in inches
    }
    
}