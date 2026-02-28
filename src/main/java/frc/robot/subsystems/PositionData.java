// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class PositionData {
    double xOffset = 0;
    double yOffset = 0;
    double lastX = 0;
    double lastY = 0;
    boolean firstPosRecieved = false;

    final DoublePublisher pidgeonYaw;

    double x = 0;
    double y = 0;
    double yaw = 0;

    final CommandSwerveDrivetrain swerve;

    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    public PositionData(
        CommandSwerveDrivetrain swerve_) {
        this.swerve = swerve_;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        pidgeonYaw = table.getDoubleTopic("auto-track-command/pidgeon-yaw").publish();
    }

    private void findOffset() {
        PoseEstimate currentPos;
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            currentPos = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
            xOffset = currentPos.pose.getMeasureX().in(Meters) - this.swerve.getState().Pose.getMeasureX().in(Meters);
            yOffset = currentPos.pose.getMeasureY().in(Meters) - this.swerve.getState().Pose.getMeasureY().in(Meters);
            return;
        }
        currentPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        xOffset = currentPos.pose.getMeasureX().in(Meters) - this.swerve.getState().Pose.getMeasureX().in(Meters);
        yOffset = currentPos.pose.getMeasureY().in(Meters) - this.swerve.getState().Pose.getMeasureY().in(Meters);
    }

    public void updatePose() {
        double correction = 0;
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            correction = 0;
        }
        pidgeonYaw.set(this.swerve.getRotation3d().getZ() * (180/Math.PI));
        if (LimelightHelpers.getTV("")) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                PoseEstimate currentPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
                Pose p = new Pose();
                p.x = currentPos.pose.getMeasureX().in(Meters);
                p.y = currentPos.pose.getMeasureY().in(Meters);
                p.yaw = currentPos.pose.getRotation().getDegrees();
                
                if (!firstPosRecieved) {
                    lastX = p.x;
                    lastY = p.y;
                    firstPosRecieved = !firstPosRecieved;
                    findOffset();
                    this.x = p.x;
                    this.y = p.y;
                    this.yaw = p.yaw;
                    return;
                }
                if ((p.x - lastX < 0.75) && (p.x - lastX > -0.75) && (p.y - lastY < 0.75) && (p.y - lastY > -0.75)) {
                    lastX = p.x;
                    lastY = p.y;
                    findOffset();
                    this.x = p.x;
                    this.y = p.y;
                    this.yaw = p.yaw;
                    return;
                }
            }
            else if (DriverStation.getAlliance().get() == Alliance.Red) {
                PoseEstimate currentPos = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
                Pose p = new Pose();
                p.x = currentPos.pose.getMeasureX().in(Meters);
                p.y = currentPos.pose.getMeasureY().in(Meters);
                p.yaw = currentPos.pose.getRotation().getDegrees();
                
                if (!firstPosRecieved) {
                    lastX = p.x;
                    lastY = p.y;
                    firstPosRecieved = !firstPosRecieved;
                    findOffset();
                    this.x = p.x;
                    this.y = p.y;
                    this.yaw = p.yaw + correction;
                    return;
                }
                if ((p.x - lastX < 0.75) && (p.x - lastX > -0.75) && (p.y - lastY < 0.75) && (p.y - lastY > -0.75)) {
                    lastX = p.x;
                    lastY = p.y;
                    findOffset();
                    this.x = p.x;
                    this.y = p.y;
                    this.yaw = p.yaw + correction;
                    return;
                }
            }

        }

        this.x = this.swerve.getState().Pose.getMeasureX().in(Meters) + xOffset;
        this.y = this.swerve.getState().Pose.getMeasureY().in(Meters) + yOffset;
        this.yaw = this.swerve.getRotation3d().getZ() * (180/Math.PI) + correction;
    }

    public Pose getPose() {
        Pose p = new Pose();
        p.x = this.x;
        p.y = this.y;
        p.yaw = this.yaw;
        return p;
    }

}
