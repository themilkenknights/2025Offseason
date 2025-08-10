package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

public class AllianceChecker {
    public static final BooleanSupplier shouldFlip =
            () -> DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;

    public static boolean isRedAlliance() {
        return shouldFlip.getAsBoolean();
    }

    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }
}
