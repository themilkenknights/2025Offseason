package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shooter.ShooterConstants.motorConfigs;
import frc.robot.util.TalonFXUtil.MotorStatusSignalSet;
import java.util.Arrays;
import java.util.stream.Stream;

public class ShooterIOTalonFX implements ShooterIO {
    private final MotorStatusSignalSet bottomLeftSignals;
    private final MotorStatusSignalSet bottomRightSignals;
    private final MotorStatusSignalSet topSignals;
    private final MotorStatusSignalSet feederSignals;
    private final MotorStatusSignalSet pivotSignals;

    // Declare motor controllers
    private final TalonFX talonBottomLeft =
            new TalonFX(ShooterConstants.bottomLeftShooterMotorId, ShooterConstants.shooterCANBus);
    private final TalonFX talonBottomRight =
            new TalonFX(ShooterConstants.bottomRightShooterMotorId, ShooterConstants.shooterCANBus);
    private final TalonFX talonTop = new TalonFX(ShooterConstants.topShooterMotorId, ShooterConstants.shooterCANBus);
    private final TalonFX talonFeeder = new TalonFX(ShooterConstants.feederMotorId, ShooterConstants.shooterCANBus);
    private final TalonFX talonPivotLeft =
            new TalonFX(ShooterConstants.pivotLeftMotorId, ShooterConstants.shooterCANBus);
    private final TalonFX talonPivotRight =
            new TalonFX(ShooterConstants.pivotRightMotorId, ShooterConstants.shooterCANBus);

    private final DigitalInput beambreak = new DigitalInput(ShooterConstants.beambreakId);

    public ShooterIOTalonFX() {
        // Initialize bottom left motor signals
        bottomLeftSignals = new MotorStatusSignalSet(talonBottomLeft, "Bottom Left", "Shooter");
        bottomRightSignals = new MotorStatusSignalSet(talonBottomRight, "Bottom Right", "Shooter");
        topSignals = new MotorStatusSignalSet(talonTop, "Top", "Shooter");
        feederSignals = new MotorStatusSignalSet(talonFeeder, "Feeder", "Shooter");
        pivotSignals = new MotorStatusSignalSet(talonPivotLeft, "Pivot", "Shooter");

        tryUntilOk(
                5,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(
                        50.0,
                        Stream.of(
                                        bottomLeftSignals.getBaseStatusSignals(),
                                        bottomRightSignals.getBaseStatusSignals(),
                                        topSignals.getBaseStatusSignals(),
                                        feederSignals.getBaseStatusSignals(),
                                        pivotSignals
                                                .getBaseStatusSignals()) // set the update frequency off all the signals
                                .flatMap(Arrays::stream)
                                .toArray(BaseStatusSignal[]::new)));
        tryUntilOk(5, () -> talonBottomLeft.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonBottomRight.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonTop.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonFeeder.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonPivotLeft.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonPivotRight.optimizeBusUtilization(0, 1.0));

        // apply the configurations
        tryUntilOk(5, () -> talonBottomLeft.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
        tryUntilOk(5, () -> talonBottomRight.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
        tryUntilOk(5, () -> talonTop.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
        tryUntilOk(5, () -> talonFeeder.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
        tryUntilOk(5, () -> talonPivotLeft.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
        tryUntilOk(5, () -> talonPivotRight.getConfigurator().apply(motorConfigs.shooterMotorConfigs));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // refresh the signals
        inputs.topMotorConnected =
                BaseStatusSignal.refreshAll(topSignals.getBaseStatusSignals()).isOK();
        inputs.bottomLeftMotorConnected = BaseStatusSignal.refreshAll(bottomLeftSignals.getBaseStatusSignals())
                .isOK();
        inputs.bottomRightMotorConnected = BaseStatusSignal.refreshAll(bottomRightSignals.getBaseStatusSignals())
                .isOK();
        inputs.feederMotorConnected = BaseStatusSignal.refreshAll(feederSignals.getBaseStatusSignals())
                .isOK();
        inputs.pivotMotorConnected =
                BaseStatusSignal.refreshAll(pivotSignals.getBaseStatusSignals()).isOK();

        // Update the connected alerts based on motor connection status
        topSignals.setConnected(inputs.topMotorConnected);
        bottomLeftSignals.setConnected(inputs.bottomLeftMotorConnected);
        bottomRightSignals.setConnected(inputs.bottomRightMotorConnected);
        feederSignals.setConnected(inputs.feederMotorConnected);
        pivotSignals.setConnected(inputs.pivotMotorConnected);

        // Update the inputs with the latest values from the signals
        inputs.topMotorInputs = topSignals.getMotorInputs();
        inputs.bottomLeftMotorInputs = bottomLeftSignals.getMotorInputs();
        inputs.bottomRightMotorInputs = bottomRightSignals.getMotorInputs();
        inputs.feederMotorInputs = feederSignals.getMotorInputs();
        inputs.pivotMotorInputs = pivotSignals.getMotorInputs();

        inputs.beambreak = beambreak.get();
    }

    private final VelocityVoltage bottomShooterRequest = new VelocityVoltage(0).withSlot(0);

    @Override
    public void setBottomShooterSpeed(AngularVelocity speed) {
        talonBottomLeft.setControl(bottomShooterRequest.withVelocity(speed));
        talonBottomRight.setControl(bottomShooterRequest.withVelocity(speed));
    }

    private final VoltageOut feederControlRequest = new VoltageOut(0);

    @Override
    public void setFeederSpeed(double volts) {
        feederControlRequest.withOutput(volts);
        talonFeeder.setControl(feederControlRequest);
    }

    private final MotionMagicVoltage pivotControlRequest = new MotionMagicVoltage(0).withSlot(0);

    @Override
    public void setTargetAngle(Angle angle) {
        talonPivotLeft.setControl(pivotControlRequest.withPosition(angle));
        talonPivotRight.setControl(pivotControlRequest.withPosition(angle));
    }

    private final VelocityVoltage topShooterRequest = new VelocityVoltage(0).withSlot(0);

    @Override
    public void setTopShooterSpeed(AngularVelocity speed) {
        talonTop.setControl(topShooterRequest.withVelocity(speed));
    }
}
