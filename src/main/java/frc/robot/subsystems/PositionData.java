// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.DoublePublisher;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class PositionData {
    double xOffset = 0;
    double yOffset = 0;

    

    final CommandSwerveDrivetrain swerve;

    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    public PositionData(
        CommandSwerveDrivetrain swerve_) {
        this.swerve = swerve_;
    }

    private void findOffset() {
        PoseEstimate currentPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        xOffset = currentPos.pose.getMeasureX().in(Meters) - this.swerve.getState().Pose.getMeasureX().in(Meters);
        yOffset = currentPos.pose.getMeasureY().in(Meters) - this.swerve.getState().Pose.getMeasureY().in(Meters);
    }

    public Pose getPoseData() {
        if (LimelightHelpers.getTV("")) {
            PoseEstimate currentPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
            Pose p = new Pose();
            p.x = currentPos.pose.getMeasureX().in(Meters);
            p.y = currentPos.pose.getMeasureY().in(Meters);
            p.yaw = currentPos.pose.getRotation().getDegrees();
            findOffset();
            return p;
        }

        Pose p = new Pose();
        p.x = this.swerve.getState().Pose.getMeasureX().in(Meters) + xOffset;
        p.y = this.swerve.getState().Pose.getMeasureY().in(Meters) + yOffset;
        p.yaw = this.swerve.getRotation3d().getZ() * (180/Math.PI);
        return p;
    }

}
