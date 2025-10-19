package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoPilotDriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.Dashboard;

public class AutopilotAutos {
    public static Command BasicL4(Drive drive, Shooter shooter) {

        return Commands.deferredProxy(() -> new SequentialCommandGroup(
                AutoPilotDriveCommands.autopilotGoToShootFromPose(drive, Dashboard.reefPosition.H)
                        .deadlineFor(shooter.prepareShot(ShooterConstants.Setpoints.L4)),
                Commands.print("Arrived at shooting position, shooting now"),
                shooter.shoot(ShooterConstants.Setpoints.L4)));
    }
}
