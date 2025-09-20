package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionDriveCommands {

    private static final double CENTER_TARGET_P = 0.02;
    private static final double CENTER_TARGET_I = 0.0;
    private static final double CENTER_TARGET_D = 0.0;
    private static final double CENTER_TARGET_TOLERANCE = 20.0;

    private static double getCenterX(PhotonTrackedTarget target) {
        return (target.minAreaRectCorners.get(0).x + target.minAreaRectCorners.get(2).x) / 2;
    }

    private static double getCenterY(PhotonTrackedTarget target) {
        return (target.minAreaRectCorners.get(0).y + target.minAreaRectCorners.get(2).y) / 2;
    }

    public static Command centerTarget(Drive drive, Supplier<PhotonTrackedTarget> target, BooleanSupplier hasTarget) {

        PIDController pid = new PIDController(CENTER_TARGET_P, CENTER_TARGET_I, CENTER_TARGET_D);
        pid.setTolerance(CENTER_TARGET_TOLERANCE);
        return new InstantCommand(() -> {})
                .alongWith(drive.run(() -> {
                            if (!hasTarget.getAsBoolean()) {

                                drive.runVelocity(new ChassisSpeeds(0, 0, 1));
                                return;
                            }
                            // pid.setP((Math.abs(getCenterY(target.get()) - 640) * 0.01));
                            drive.runVelocity(
                                    new ChassisSpeeds(0, 0, pid.calculate(getCenterX(target.get()), 640 / 2)));
                        })
                        .until(pid::atSetpoint));
    }

    public static Command autoIntake(
            Drive drive, Intake intake, Supplier<PhotonTrackedTarget> target, BooleanSupplier hasTarget) {
        PIDController pid = new PIDController(CENTER_TARGET_P, CENTER_TARGET_I, CENTER_TARGET_D);
        pid.setTolerance(CENTER_TARGET_TOLERANCE);
        return centerTarget(drive, target, hasTarget)
                .andThen(drive.run(() -> {
                            drive.runVelocity(
                                    new ChassisSpeeds(-0.5, 0, pid.calculate(getCenterX(target.get()), 640 / 2)));
                        })
                        .withDeadline(intake.intakeCoral()));
    }
}
