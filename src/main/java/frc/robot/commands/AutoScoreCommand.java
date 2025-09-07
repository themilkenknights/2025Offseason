package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.Dashboard.Target;

public class AutoScoreCommand {
    public static Command AutoScore(Target target, Drive drive, Shooter shooter) {
        final ShooterConstants.Setpoint setpoint = ShooterConstants.getSetpointFromLevel(target.getLevel());
        return new ParallelDeadlineGroup(
                        AutoPilotDriveCommands.autopilotGoToShootFromPose(drive, target.getPosition()),
                        shooter.prepareShot(setpoint))
                .andThen(shooter.shoot(setpoint));
    }
}
