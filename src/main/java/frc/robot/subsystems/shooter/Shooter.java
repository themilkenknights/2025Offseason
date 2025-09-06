package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.ShooterConstants.Setpoint;
import frc.robot.subsystems.shooter.ShooterConstants.Setpoints;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    @AutoLogOutput(key = "Shooter/goalAngle")
    private Angle goalAngle = Degrees.of(0); // TODO: make a constant for this

    @AutoLogOutput(key = "Shooter/topRollerSpeed")
    private AngularVelocity topRollerSpeed = RotationsPerSecond.of(0);

    @AutoLogOutput(key = "Shooter/bottomRollerSpeed")
    private AngularVelocity bottomRollerSpeed = RotationsPerSecond.of(0);

    private void setShooterGoalAngle(Angle angle) {
        this.goalAngle = angle;
        io.setTargetAngle(angle);
    }

    private void setTopRollerSpeed(AngularVelocity speed) {
        this.topRollerSpeed = speed;
        io.setTopShooterSpeed(speed);
    }

    private void setBottomRollerSpeed(AngularVelocity speed) {
        this.bottomRollerSpeed = speed;
        io.setBottomShooterSpeed(speed);
    }

    @AutoLogOutput(key = "Shooter/atSetpoint")
    public boolean atPivotSetpoint() {
        return Radians.of(inputs.pivotLeftMotorInputs.positionRads())
                .isNear(goalAngle, ShooterConstants.pivotTolerance);
    }

    public Angle getPivotAngle() {
        return Radians.of(-inputs.pivotLeftMotorInputs.positionRads());
    }

    @AutoLogOutput(key = "Shooter/topRollerAtSetpoint")
    public boolean atTopRollerSetpoint() {
        return (RadiansPerSecond.of(inputs.topMotorInputs.velocityRadsPerSec())
                .isNear(topRollerSpeed, ShooterConstants.rollerSpeedTolerance));
    }

    @AutoLogOutput(key = "Shooter/bottomRollerAtSetpoint")
    public boolean atBottomRollerSetpoint() {
        return (RadiansPerSecond.of(inputs.bottomLeftMotorInputs.velocityRadsPerSec())
                        .isNear(bottomRollerSpeed, ShooterConstants.rollerSpeedTolerance)
                && RadiansPerSecond.of(inputs.bottomRightMotorInputs.velocityRadsPerSec())
                        .isNear(bottomRollerSpeed, ShooterConstants.rollerSpeedTolerance));
    }

    @AutoLogOutput(key = "Shooter/readyToShoot")
    public boolean readyToShoot() {
        return atPivotSetpoint() && atTopRollerSetpoint() && atBottomRollerSetpoint();
    }

    public Command goToAngle(Setpoint goal) {
        return runOnce(() -> setShooterGoalAngle(goal.pivotAngle()))
                .andThen(new WaitUntilCommand(this::atPivotSetpoint));
    }

    public Command rampUp(Setpoint goal) {
        return runOnce(() -> {
                    setTopRollerSpeed(goal.topWheelSpeed());
                    setBottomRollerSpeed(goal.bottomWheelSpeed());
                })
                .andThen(new WaitUntilCommand(() -> (atTopRollerSetpoint() && atBottomRollerSetpoint())));
    }

    public Command prepareShot(Setpoint goal) {
        return runOnce(() -> {
                    setTopRollerSpeed(goal.topWheelSpeed());
                    setBottomRollerSpeed(goal.bottomWheelSpeed());
                    setShooterGoalAngle(goal.pivotAngle());
                })
                .andThen(new WaitUntilCommand(this::readyToShoot));
    }

    public Command shoot(Setpoint goal) {
        return prepareShot(goal)
                .andThen(startEnd(
                                () -> {
                                    io.setFeederSpeed(ShooterConstants.feederSpeedShooting);
                                },
                                () -> {
                                    io.setFeederSpeed(0);
                                    setBottomRollerSpeed(RotationsPerSecond.of(0));
                                    setTopRollerSpeed(RotationsPerSecond.of(0));
                                })
                        .raceWith(new WaitUntilCommand(() -> {
                                    return !getBeambreak();
                                })
                                .andThen(new WaitCommand(ShooterConstants.shootingTime))));
    }

    public Command goToLoadingSetpoint() {
        return goToAngle(Setpoints.Load.getSetpoint());
    }

    public Command loadCoral() {
        return goToAngle(Setpoints.Load.getSetpoint())
                .andThen(startEnd(
                                () -> {
                                    io.setFeederSpeed(ShooterConstants.feederSpeedLoading);
                                },
                                () -> {
                                    io.setFeederSpeed(0);
                                })
                        .until(this::getBeambreak));
    }

    @AutoLogOutput(key = "Shooter/beambreak")
    public boolean getBeambreak() {
        return inputs.beambreak;
    }
}
