package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TalonFXUtil.MotorInputs;

public class ShooterIOSim implements ShooterIO {

  private AngularVelocity bottomShooterSpeed = RotationsPerSecond.zero();
  private AngularVelocity topShooterSpeed = RotationsPerSecond.zero();

  private ProfiledPIDController pivotDummyController =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              ShooterConstants.motorConfigs
                  .shooterMotorConfigs
                  .MotionMagic
                  .MotionMagicCruiseVelocity,
              ShooterConstants.motorConfigs
                  .shooterMotorConfigs
                  .MotionMagic
                  .MotionMagicAcceleration));

  @Override
  public void setBottomShooterSpeed(AngularVelocity speed) {
   bottomShooterSpeed = speed;
  }

  private Voltage feederSpeed = Volts.zero();
  @Override
  public void setFeederSpeed(Voltage speed) {
    feederSpeed = speed;
  }

  @Override
  public void setSimGoal(SimGoal simGoal) {
    // TODO: maple sim shooting
  }

  @Override
  public void setTargetAngle(Angle angle) {
    pivotDummyController.setGoal(angle.in(Rotations));
  }

  @Override
  public void setTopShooterSpeed(AngularVelocity speed) {
    topShooterSpeed = speed;
  }

  Angle currentAngle = Degrees.zero();
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    //run the fake PID controller to simulate the pivot motor
    pivotDummyController.calculate(currentAngle.in(Rotations));
    currentAngle = Rotation.of(pivotDummyController.getSetpoint().position);
    //pretend the motors are connected
    inputs.bottomLeftMotorConnected = true;
    inputs.bottomRightMotorConnected = true;
    inputs.topMotorConnected = true;
    inputs.feederMotorConnected = true;
    inputs.pivotLeftMotorConnected = true;
    inputs.pivotRightMotorConnected = true;

    //pretend the motors are at their setpoints
    inputs.bottomLeftMotorInputs =
        new MotorInputs(
            0.0,
            bottomShooterSpeed.in(RotationsPerSecond),
            0.0,
            0.0,
            0.0,
            0.0,
            false);
    inputs.bottomRightMotorInputs =
        new MotorInputs(
            0.0,
            bottomShooterSpeed.in(RotationsPerSecond),
            0.0,
            0.0,
            0.0,
            0.0,
            false);

    inputs.topMotorInputs =
        new MotorInputs(
            0.0,
            topShooterSpeed.in(RotationsPerSecond),
            0.0,
            0.0,
            0.0,
            0.0,
            false);
    inputs.pivotRightMotorInputs =
        new MotorInputs(
            currentAngle.in(Radians),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            false);
    inputs.pivotLeftMotorInputs =
        new MotorInputs(
            currentAngle.in(Radians),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            false);
    inputs.feederMotorInputs =
        new MotorInputs(
            0.0,
            0.0,
            feederSpeed.in(Volts),
            0.0,
            0.0,
            0.0,
            false);

  }
}
