package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.TalonFXUtil.MotorInputs;
import java.util.function.BooleanSupplier;

public class ShooterIOSim implements ShooterIO {

    private final DigitalInput beambreak = new DigitalInput(ShooterConstants.beambreakId);

    private final BooleanSupplier obtainGamePiece;

    public ShooterIOSim(BooleanSupplier obtainGamePiece) {
        this.obtainGamePiece = obtainGamePiece;
    }

    private boolean hasCoral = false;

    private AngularVelocity bottomShooterSpeed = RotationsPerSecond.zero();
    private AngularVelocity topShooterSpeed = RotationsPerSecond.zero();
    private ProfiledPIDController pivotDummyController = new ProfiledPIDController(
            2,
            0,
            0,
            new Constraints(
                    3, // ShooterConstants.motorConfigs.shooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity,
                    3)); // ShooterConstants.motorConfigs.shooterMotorConfigs.MotionMagic.MotionMagicAcceleration));

    @Override
    public void setBottomShooterSpeed(AngularVelocity speed) {
        bottomShooterSpeed = speed;
    }

    private Voltage feederSpeed = Volts.zero();

    @Override
    public void setFeederSpeed(double speed) {
        feederSpeed = Volts.of(speed);
    }

    @Override
    public void setSimGoal(SimGoal simGoal) {
        // TODO: maple sim shooting
    }

    @Override
    public void setTargetAngle(Angle angle) {
        pivotDummyController.reset(currentAngle.in(Rotation));
        pivotDummyController.setGoal(angle.in(Rotations));
    }

    @Override
    public void setTopShooterSpeed(AngularVelocity speed) {
        topShooterSpeed = speed;
    }

    Angle currentAngle = Degrees.zero();

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // maplesim stuff
        if (feederSpeed.in(Volts) > 0 && !hasCoral) {
            hasCoral = obtainGamePiece.getAsBoolean();
        }
        inputs.beambreak = !hasCoral;

        // run the fake PID controller to simulate the pivot motor and simulator
        pivotDummyController.calculate(currentAngle.in(Rotation));

        currentAngle = Rotations.of(pivotDummyController.getSetpoint().position);

        // pretend the motors are connected
        inputs.bottomLeftMotorConnected = true;
        inputs.bottomRightMotorConnected = true;
        inputs.topMotorConnected = true;
        inputs.feederMotorConnected = true;
        inputs.pivotLeftMotorConnected = true;
        inputs.pivotRightMotorConnected = true;

        // simulate delay in responsiveness of the flywheels using the SlewRateLimiter
        AngularVelocity bottomShooterMeasuredSpeed = RotationsPerSecond.of(
                ShooterConstants.bottomWheelLimiter.calculate(bottomShooterSpeed.in(RotationsPerSecond)));
        AngularVelocity topShooterMeasuredSpeed = RotationsPerSecond.of(
                ShooterConstants.topWheelLimiter.calculate(topShooterSpeed.in(RotationsPerSecond)));
        // pretend the motors are at their setpoints
        inputs.bottomLeftMotorInputs =
                new MotorInputs(0.0, bottomShooterMeasuredSpeed.in(RadiansPerSecond), 0.0, 0.0, 0.0, 0.0, false);
        inputs.bottomRightMotorInputs =
                new MotorInputs(0.0, bottomShooterMeasuredSpeed.in(RadiansPerSecond), 0.0, 0.0, 0.0, 0.0, false);

        inputs.topMotorInputs =
                new MotorInputs(0.0, topShooterMeasuredSpeed.in(RadiansPerSecond), 0.0, 0.0, 0.0, 0.0, false);
        inputs.pivotRightMotorInputs = new MotorInputs(currentAngle.in(Radians), 0.0, 0.0, 0.0, 0.0, 0.0, false);
        inputs.pivotLeftMotorInputs = new MotorInputs(currentAngle.in(Radians), 0.0, 0.0, 0.0, 0.0, 0.0, false);
        inputs.feederMotorInputs = new MotorInputs(0.0, 0.0, feederSpeed.in(Volts), 0.0, 0.0, 0.0, false);
    }
}
