package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;


public class LimelightVisionSubsystem extends SubsystemBase {
    private final AnglerSubsytem angler = new AnglerSubsytem();

    private final String limeLightName = "limelight"; // change this to the limelight's name

    public LimelightVisionSubsystem(int pipeline) {
        LimelightHelpers.setPipelineIndex(limeLightName, pipeline);
    }

    public double getTx() {
        return LimelightHelpers.getTX(limeLightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limeLightName);
    }

    public double getTa() {
        return LimelightHelpers.getTA(limeLightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limeLightName);
    }
        
    public double getTxnc() {
        return LimelightHelpers.getTXNC(limeLightName);
    }

    public double getTync() {
        return LimelightHelpers.getTYNC(limeLightName);
    }

    public int getTagID() {
        if (!hasTarget()) {
            // -1 is no target
            return -1;
        }
        return (int) LimelightHelpers.getFiducialID(limeLightName);
    }

    public void setAnglerToTag() {
        if (!hasTarget()) {
            return;
        }

        Double ty = getTy();
        Angle angle = Degrees.of(ty);

        //angler.setAnglerPositionCommand(angle);
    }

    public Command setAnglerToTagCommand() {
        return runOnce(() -> setAnglerToTag());
    }

}

