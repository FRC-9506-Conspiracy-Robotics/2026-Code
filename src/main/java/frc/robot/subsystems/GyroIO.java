// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double odometryYawTimestamps = 0.0;
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    class GyroIOInputsAutoLogged extends GyroIOInputs implements LoggableInputs {
        public void toLog(LogTable table) {
            table.put("connectedg", connected);
            table.put("yawPosition", yawPosition);
            table.put("yawVelocityRadPerSec", yawVelocityRadPerSec);
            table.put("odometryYawTimestamps", odometryYawTimestamps);
            table.put("odometryYawPositions", odometryYawPositions);
        }

        public void fromLog(LogTable table) {
            connected = table.get("connectedg", connected);
            yawPosition = table.get("yawPosition", yawPosition);
            yawVelocityRadPerSec = table.get("yawVelocityRadPerSec", yawVelocityRadPerSec);
            odometryYawTimestamps = table.get("odometryYawTimestamps", odometryYawTimestamps);
            odometryYawPositions = table.get("odometryYawPositions", odometryYawPositions);
        }
    }

    public default void updateInputs(GyroIOInputs inputs) {};
}
