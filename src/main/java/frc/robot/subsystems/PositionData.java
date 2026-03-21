// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightNames;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;

/** Add your docs here. */
public class PositionData {

    final DoublePublisher pidgeonYaw;
    final DoublePublisher allianceFlip;

    public static double speedFactor = 1;

    double x = 0;
    double y = 0;
    double yaw = 0;
    double velX = 0;
    double velY = 0;

    final SwerveDrive swerve;

    public class Pose {
        public double x;
        public double y;
        public double yaw;
        public double velX = 0;
        public double velY = 0;
    }

    public PositionData(
        SwerveDrive swerve_) {
        this.swerve = swerve_;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        pidgeonYaw = table.getDoubleTopic("auto-track-command/pidgeon-yaw").publish();
        this.allianceFlip = table.getDoubleTopic("auto-track-command/alliance-flipped").publish();
    }

    public void updatePose() {
        
        pidgeonYaw.set(this.swerve.getGyroRotation3d().getZ() * (180/Math.PI));

        ChassisSpeeds velocity = this.swerve.getFieldVelocity();
        this.velX = velocity.vxMetersPerSecond;
        this.velY = velocity.vyMetersPerSecond;

        boolean frontVis = LimelightHelpers.getTV(LimelightNames.limelight4AFront);
        boolean leftVis = LimelightHelpers.getTV(LimelightNames.limelight3ALeft);
        boolean rightVis = LimelightHelpers.getTV(LimelightNames.limelight3ARight);
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        int validTargets = 0;
        PoseEstimate frontPose = null;
        PoseEstimate leftPose = null;
        PoseEstimate rightPose = null;
        double estimatedX = 0;
        double estimatedY = 0;
        if (frontVis) {
            validTargets += 1;
            if (isRedAlliance) {
                frontPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LimelightNames.limelight4AFront);
            }
            else {
                frontPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightNames.limelight4AFront);
            }
            if (frontPose.pose.getMeasureX().in(Meters) == 0 && frontPose.pose.getMeasureY().in(Meters) == 0) {
                validTargets += -1;
                frontVis = false;
            }
        }
        if (leftVis) {
            validTargets += 1;
            if (isRedAlliance) {
                leftPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LimelightNames.limelight3ALeft);
            }
            else {
                leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightNames.limelight3ALeft);
            }
            if (leftPose.pose.getMeasureX().in(Meters) == 0 && leftPose.pose.getMeasureY().in(Meters) == 0) {
                validTargets += -1;
                leftVis = false;
            }
        }
        if (rightVis) {
            validTargets += 1;
            if (isRedAlliance) {
                rightPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LimelightNames.limelight3ARight);
            }
            else {
                rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightNames.limelight3ARight);
            }
            if (rightPose.pose.getMeasureX().in(Meters) == 0 && rightPose.pose.getMeasureY().in(Meters) == 0) {
                validTargets += -1;
                rightVis = false;
            }
        }
        if (leftVis || frontVis || rightVis) {
            if (frontVis) {
                estimatedX += frontPose.pose.getMeasureX().in(Meters);
                estimatedY += frontPose.pose.getMeasureY().in(Meters);
            }
            if (leftVis) {
                estimatedX += leftPose.pose.getMeasureX().in(Meters);
                estimatedY += leftPose.pose.getMeasureY().in(Meters);
            }
            if (rightVis) {
                estimatedX += rightPose.pose.getMeasureX().in(Meters);
                estimatedY += rightPose.pose.getMeasureY().in(Meters);
            }
            estimatedX = estimatedX / validTargets;
            estimatedY = estimatedY / validTargets;

            this.x = estimatedX;
            this.y = estimatedY;
            this.yaw = this.swerve.getGyroRotation3d().getZ() * 180/Math.PI;
            Pose2d newPose = new Pose2d(estimatedX, estimatedY, this.swerve.getYaw());
            swerve.resetOdometry(newPose);

            return;
        }

        this.x = this.swerve.getPose().getMeasureX().in(Meters);
        this.y = this.swerve.getPose().getMeasureY().in(Meters);
        this.yaw = this.swerve.getGyroRotation3d().getZ() * (180/Math.PI);
    }

    public Pose getPose() {
        Pose p = new Pose();
        p.x = this.x;
        p.y = this.y;
        p.yaw = this.yaw;
        p.velX = this.velX;
        p.velY = this.velY;
        return p;
    }

}