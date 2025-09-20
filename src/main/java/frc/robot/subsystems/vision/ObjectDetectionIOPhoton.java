package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class ObjectDetectionIOPhoton implements ObjectDetectionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public ObjectDetectionIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        inputs.latestTargetObservation = camera.getLatestResult();
    }
}
