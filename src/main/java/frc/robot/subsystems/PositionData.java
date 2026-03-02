// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
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
    double xOffset = 0;
    double yOffset = 0;
    double lastX = 0;
    double lastY = 0;
    boolean firstPosRecieved = false;

    final DoublePublisher pidgeonYaw;
    final DoublePublisher allianceFlip;

    double x = 0;
    double y = 0;
    double yaw = 0;

    final SwerveDrive swerve;

    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    public PositionData(
        SwerveDrive swerve_) {
        this.swerve = swerve_;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        pidgeonYaw = table.getDoubleTopic("auto-track-command/pidgeon-yaw").publish();
        this.allianceFlip = table.getDoubleTopic("auto-track-command/alliance-flipped").publish();
    }

    private void findOffset(double visX, double visY) {
        xOffset = visX - this.swerve.getPose().getMeasureX().in(Meters);
        yOffset = visY - this.swerve.getPose().getMeasureY().in(Meters);
    }

    public void updatePose() {
        double correction = 0;
        double allianceFlipped = 1;
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            correction = 0;
            allianceFlipped = -1;
        }
        this.allianceFlip.set(allianceFlipped);
        pidgeonYaw.set(this.swerve.getGyroRotation3d().getZ() * (180/Math.PI));
        boolean frontVis = LimelightHelpers.getTV(LimelightNames.limelight4AFront);
        boolean leftVis = LimelightHelpers.getTV(LimelightNames.limelight3ALeft);
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        int validTargets = 0;
        PoseEstimate frontPose = null;
        PoseEstimate leftPose = null;
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
        }
        if (leftVis) {
            validTargets += 1;
            if (isRedAlliance) {
                leftPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LimelightNames.limelight3ALeft);
            }
            else {
                leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightNames.limelight3ALeft);
            }
        }
        if (leftVis || frontVis) {
            if (frontVis) {
                estimatedX += frontPose.pose.getMeasureX().in(Meters);
                estimatedY += frontPose.pose.getMeasureY().in(Meters);
            }
            if (leftVis) {
                estimatedX += leftPose.pose.getMeasureX().in(Meters);
                estimatedY += leftPose.pose.getMeasureY().in(Meters);
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

        this.x = this.swerve.getPose().getMeasureX().in(Meters) + xOffset;
        this.y = this.swerve.getPose().getMeasureY().in(Meters) + yOffset;
        this.yaw = this.swerve.getGyroRotation3d().getZ() * (180/Math.PI) + correction;
    }

    public Pose getPose() {
        Pose p = new Pose();
        p.x = this.x;
        p.y = this.y;
        p.yaw = this.yaw;
        return p;
    }

}