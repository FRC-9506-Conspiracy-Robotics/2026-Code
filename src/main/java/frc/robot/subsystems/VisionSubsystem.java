package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("photonvision");

    public VisionSubsystem() {}

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return Optional.of(result.getBestTarget());
        } else {
            return Optional.empty();
        }
    }
    
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }
}
