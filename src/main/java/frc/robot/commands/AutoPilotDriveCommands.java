package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Dashboard;
import frc.robot.util.Dashboard.reefPosition;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AutoPilotDriveCommands {
    // Autopilot constants
    private static final APConstraints constraints = new APConstraints(10, 10);
    private static final APProfile profile = new APProfile(constraints)
            .withErrorXY(Centimeters.of(3))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(16));
    private static final Autopilot autopilot = new Autopilot(profile);

    private AutoPilotDriveCommands() {
        // Prevent instantiation
    }

    public static Command autopilotGoToShootFromPose(Drive drive, reefPosition position) {
        return autopilotDriveCommand(
                new APTarget(Dashboard.getPoseFromReefPosition(position))
                        .withEntryAngle(drive.getPose().getRotation()),
                drive);
    }

    public static Command testDriveCommand(Drive drive) {

        return autopilotDriveCommand(
                new APTarget(new Pose2d(3, 2, new Rotation2d())).withEntryAngle(Rotation2d.k180deg), drive);
    }

    public static Command goToNearestShootFromPose(Drive drive) {
        return drive.defer(() -> {
            return autopilotDriveCommand(
                    new APTarget(FieldConstants.getClosestShootFromPose(drive.getPose()))
                            .withEntryAngle(drive.getPose().getRotation()),
                    drive);
        });
    }

    public static Command autopilotDriveCommand(APTarget goal, Drive drive) {
        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                DriveCommands.ANGLE_KP,
                0.0,
                DriveCommands.ANGLE_KD,
                new TrapezoidProfile.Constraints(
                        DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return drive.run(() -> {
                    Pose2d pose = drive.getPose();
                    APResult output = autopilot.calculate(pose, drive.getChassisSpeeds(), goal);

                    double omega = angleController.calculate(
                            drive.getRotation().getRadians(),
                            output.targetAngle().getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds outputSpeeds = new ChassisSpeeds(
                            output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond), omega);
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(outputSpeeds, drive.getRotation()));

                    Logger.recordOutput("Autodrive/goalPose", goal.getReference());
                })
                .until(() -> autopilot.atTarget(drive.getPose(), goal))
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
                .andThen(() -> drive.stop());
    }
}
