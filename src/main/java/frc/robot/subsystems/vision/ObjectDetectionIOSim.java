package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class ObjectDetectionIOSim extends ObjectDetectionIOPhoton {

    private final PhotonCameraSim cameraSim;
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;

    public ObjectDetectionIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;
        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("objects");
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties().setFPS(24);
        cameraProperties.setCalibration(640, 640, Rotation2d.fromDegrees(90));

        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        super.updateInputs(inputs);

        visionSim.clearVisionTargets();
        SimulatedArena.getInstance().getGamePiecesByType("Coral").forEach(object -> {
            visionSim.addVisionTargets(
                    "Coral",
                    new VisionTargetSim(
                            object.getPose3d(),
                            new TargetModel(
                                    Inches.of(11.75).in(Meters),
                                    Inches.of(4.5).in(Meters),
                                    Inches.of(4.5).in(Meters))));
        });
        SimulatedArena.getInstance().getGamePiecesByType("CoralAlgaeStack").forEach(object -> {
            visionSim.addVisionTargets(
                    "Coral",
                    new VisionTargetSim(
                            object.getPose3d(),
                            new TargetModel(
                                    Inches.of(4.5).in(Meters),
                                    Inches.of(4.5).in(Meters),
                                    Inches.of(11.75).in(Meters))));
        });

        visionSim.update(poseSupplier.get());
    }
}
