package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Dashboard extends SubsystemBase {
    public static enum Level {
        L1,
        L2,
        L3,
        L4
    }

    public static enum reefPosition {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L
    }

    public static class Target {
        private final Level level;
        private final reefPosition position;

        public Target(Level level, reefPosition position) {
            this.level = level;
            this.position = position;
        }

        public Level getLevel() {
            return level;
        }

        public reefPosition getPosition() {
            return position;
        }
    }

    public static Target getSelectedTarget() {
        return new Target(levelChooser.get(), poseChooser.get());
    }

    public static Pose2d getPoseFromReefPosition(reefPosition position) {
        return FieldConstants.getCurrentAllianceShootFromPositions()[position.ordinal()];
    }

    public Field2d dashboardField = new Field2d();

    final Supplier<Pose2d> robotPose;

    private static final LoggedDashboardChooser<Level> levelChooser =
            new LoggedDashboardChooser<>("Dashboard/SelectedLevel");
    private static final LoggedDashboardChooser<reefPosition> poseChooser =
            new LoggedDashboardChooser<>("Dashboard/SelectedSpot");

    public Dashboard(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        SmartDashboard.putData(dashboardField);

        for (Level level : Level.values()) {
            levelChooser.addOption(level.name(), level);
        }
        for (reefPosition pos : reefPosition.values()) {
            poseChooser.addOption(pos.name(), pos);
        }
        poseChooser.addDefaultOption(reefPosition.A.toString(), reefPosition.A);
        levelChooser.addDefaultOption(Level.L4.toString(), Level.L4);
    }

    @Override
    public void periodic() {
        dashboardField.setRobotPose(robotPose.get());
        dashboardField.getObject("ShootFromPose").setPose(getPoseFromReefPosition(poseChooser.get()));
    }
}
