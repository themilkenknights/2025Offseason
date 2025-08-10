package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
    // All constants are for the blue alliance

    public static AprilTagFieldLayout aprilTagFieldLayout = VisionConstants.aprilTagLayout;

    public static final Distance reefSideWidth = Inches.of(65.5);
    public static final Distance reefApothem = reefSideWidth.div(2).div(Math.tan(Math.toRadians(45)));
    public static final Distance reefToAllianceWall = Inches.of(144);
    public static final Distance fieldSideWallToReefCenter = Inches.of(158.500049);

    public static final Distance reefZoneRadius = Inches.of(93.5); // is this right?

    public static final Distance shootFromOffset = Inches.of(30);

    public static final Pose2d reefCenterPointOnField =
            new Pose2d(reefSideWidth.div(2).plus(reefToAllianceWall), fieldSideWallToReefCenter, Rotation2d.k180deg);

    public static final Distance reefPoleCenterToCenter = Centimeters.of(33.02);

    public static enum reefPositions {
        A(1),
        B(2),
        C(3),
        D(4),
        E(5),
        F(6),
        G(7),
        H(8),
        I(9),
        J(10),
        K(11),
        L(12);
        private Pose2d polePose;
        private Pose2d shootFromPose;

        public Pose2d getPolePose() {
            return polePose;
        }

        public Pose2d getShootFromPose() {
            return shootFromPose;
        }

        private reefPositions(int poleNum) {
            int isLeftPole =
                    (poleNum - 1) % 2 == 0 ? -1 : 1; // A, C, E, G, I, K are left poles. +1 if left, -1 if right
            Translation2d standardPoleTransform = new Translation2d( // From center of reef to A/B
                    reefApothem, reefPoleCenterToCenter.div(2).times(isLeftPole));

            polePose = reefCenterPointOnField.plus(new Transform2d(standardPoleTransform, Rotation2d.k180deg));
            shootFromPose = polePose.transformBy(
                    new Transform2d(new Translation2d(shootFromOffset.unaryMinus(), Meters.zero()), Rotation2d.kZero));

            polePose = polePose.rotateAround(
                    reefCenterPointOnField.getTranslation(), new Rotation2d(((poleNum - 1) / 2) * Math.PI / 3));
            shootFromPose = shootFromPose.rotateAround(
                    reefCenterPointOnField.getTranslation(), new Rotation2d(((poleNum - 1) / 2) * Math.PI / 3));
        }
    }

    private static final Pose2d[] shootFromPositions = Arrays.stream(reefPositions.values())
            .map(reefPositions::getShootFromPose)
            .toArray(Pose2d[]::new);

    public static Pose2d[] getCurrentAllianceShootFromPositions() {
        Pose2d[] poses = Arrays.stream(shootFromPositions)
                .map(FieldMirroringUtils::toCurrentAlliancePose)
                .toArray(Pose2d[]::new);
        Logger.recordOutput("Autodrive/CurrentAllianceShootFromPositions", poses);
        Logger.recordOutput(
                "Autodrive/bluePolePoses",
                Arrays.stream(reefPositions.values())
                        .map(reefPositions::getPolePose)
                        .toArray(Pose2d[]::new));
        return poses;
    }

    public static Pose2d getClosestShootFromPose(Pose2d pose) {

        return pose.nearest(List.of(getCurrentAllianceShootFromPositions()));
    }
}
