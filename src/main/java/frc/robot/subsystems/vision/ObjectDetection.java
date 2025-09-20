package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
    private final ObjectDetectionIO io;
    private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();

    private boolean hasTarget = false;
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();

    public boolean hasTarget() {
        return hasTarget;
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public ObjectDetection(ObjectDetectionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectDetection", inputs);
        hasTarget = inputs.latestTargetObservation.hasTargets();
        if (inputs.connected && hasTarget) {
            Logger.recordOutput(
                    "Object Detection/Best target x",
                    inputs.latestTargetObservation
                            .getBestTarget()
                            .getDetectedCorners()
                            .get(0)
                            .x);
            bestTarget = inputs.latestTargetObservation.getBestTarget();
        }
    }
}
