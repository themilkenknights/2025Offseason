package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class FeederCommands {

    public static Command feed(Intake intake, Shooter shooter) {
        return new SequentialCommandGroup(
                shooter.goToLoadingSetpoint(), new ParallelDeadlineGroup(shooter.loadCoral(), intake.feedCoral()));
    }
}
