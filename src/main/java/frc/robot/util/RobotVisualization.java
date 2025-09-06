package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotVisualization extends SubsystemBase {

    public static final Translation3d zeroedPivotTranslation =
            new Translation3d(Inches.of(-1), Meters.zero(), Inches.of(12.75));
    public Supplier<Angle> pivotAngle;

    public RobotVisualization(Supplier<Angle> pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Logger.recordOutput("RobotVisualization/zeroPose", new Pose2d());
        Logger.recordOutput("RobotVisualization/Components", new Pose3d[] {
            new Pose3d(
                    zeroedPivotTranslation, new Rotation3d(0, pivotAngle.get().in(Radian), 0))
        });
    }
}
