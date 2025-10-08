package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    @AutoLogOutput(key = "Intake/goalAngle")
    private Angle goalAngle = Degrees.zero();

    private void setGoalAngle(Angle angle) {
        this.goalAngle = angle;
        io.setPivotTargetAngle(angle);
    }

    @AutoLogOutput(key = "Intake/atGoalAngle")
    public boolean atPivotSetpoint() {
        return Radians.of(inputs.pivotMotorInputs.positionRads())
                .isNear(goalAngle, IntakeConstants.getPivotTolerance());
    }

    public Angle getCurrentAngle() {
        return Radians.of(inputs.pivotMotorInputs.positionRads());
    }

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Trigger atSetpoint() {
        return new Trigger(() -> atPivotSetpoint());
    }

    private Command goToPivotSetpoint(Angle angle) {
        return startEnd(() -> setGoalAngle(angle), () -> {}).until(() -> atPivotSetpoint());
    }

    public Command intakeCoral() {
        return defer(() -> goToPivotSetpoint(IntakeConstants.getPivotExtendedAngle()))
                .andThen(startEnd(
                                () -> {
                                    io.setFrontRollerSpeed(IntakeConstants.intakeingVoltage);
                                    io.setIndexerSpeed(IntakeConstants.intakeingVoltage);
                                },
                                () -> {
                                    io.setFrontRollerSpeed(0);
                                    io.setIndexerSpeed(0);
                                })
                        .until(() -> !inputs.beambreak));
    }

    /**
     * Moves the pivot to the feeding setpoint and turns on the indexer rollers.<br>
     *
     * <h1>The command never finishes</h1>
     *
     * and must be interrupted to stop the indexer.
     *
     * @return A {@link Command} that executes the feeding sequence.
     */
    public Command feedCoral() {
        return defer(() -> goToPivotSetpoint(IntakeConstants.getPivotFeedingAngle()))
                .andThen(startEnd(
                        () -> {
                            io.setIndexerSpeed(IntakeConstants.feedingVoltage);
                        },
                        () -> {
                            io.setFrontRollerSpeed(0);
                            io.setIndexerSpeed(0);
                        }));
    }

    /**
     * Creates a command to stow the intake mechanism. This command sets the intake pivot to the stowed angle and stops
     * the front roller and indexer motors. Finishes instantly.
     *
     * @return A command that performs the stowing operation.
     */
    public Command stow() {
        return defer(() -> runOnce(() -> {
            setGoalAngle(IntakeConstants.getPivotStowedAngle());
            io.setFrontRollerSpeed(0);
            io.setIndexerSpeed(0);
        }));
    }
}
