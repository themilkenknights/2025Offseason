package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IntakeConstants.motorConfigs;
import frc.robot.util.TalonFXUtil.MotorStatusSignalSet;
import java.util.Arrays;
import java.util.stream.Stream;

public class IntakeIOTalonFX implements IntakeIO {
    private final MotorStatusSignalSet frontRollerSignals;
    private final MotorStatusSignalSet indexerSignals;
    private final MotorStatusSignalSet pivotSignals;

    private final DigitalInput beambreak = new DigitalInput(IntakeConstants.beambreakPort);

    private final TalonFX talonFrontRoller =
            new TalonFX(IntakeConstants.frontRollerMotorId, IntakeConstants.intakeCANBus);
    private final TalonFX talonIndexer = new TalonFX(IntakeConstants.indexerMotorId, IntakeConstants.intakeCANBus);
    private final TalonFX talonPivot = new TalonFX(IntakeConstants.pivotMotorId, IntakeConstants.intakeCANBus);

    public IntakeIOTalonFX() {
        frontRollerSignals = new MotorStatusSignalSet(talonFrontRoller, "Front Roller", "Intake");
        indexerSignals = new MotorStatusSignalSet(talonIndexer, "Indexer", "Intake");
        pivotSignals = new MotorStatusSignalSet(talonPivot, "Pivot", "Intake");

        tryUntilOk(
                5,
                () -> BaseStatusSignal.setUpdateFrequencyForAll(
                        50.0,
                        Stream.of(
                                        frontRollerSignals.getBaseStatusSignals(),
                                        indexerSignals.getBaseStatusSignals(),
                                        pivotSignals.getBaseStatusSignals())
                                .flatMap(Arrays::stream)
                                .toArray(BaseStatusSignal[]::new)));
        tryUntilOk(5, () -> talonFrontRoller.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonIndexer.optimizeBusUtilization(0, 1.0));
        tryUntilOk(5, () -> talonPivot.optimizeBusUtilization(0, 1.0));

        tryUntilOk(5, () -> talonFrontRoller.getConfigurator().apply(motorConfigs.frontRollerMotor));
        tryUntilOk(5, () -> talonIndexer.getConfigurator().apply(motorConfigs.indexerMotor));
        tryUntilOk(5, () -> talonPivot.getConfigurator().apply(motorConfigs.pivotMotor));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.frontRollerIntakeMotorConnected = BaseStatusSignal.refreshAll(frontRollerSignals.getBaseStatusSignals())
                .isOK();
        inputs.indexerIntakeMotorConnected = BaseStatusSignal.refreshAll(indexerSignals.getBaseStatusSignals())
                .isOK();
        inputs.pivotMotorConnected =
                BaseStatusSignal.refreshAll(pivotSignals.getBaseStatusSignals()).isOK();

        frontRollerSignals.setConnected(inputs.frontRollerIntakeMotorConnected);
        indexerSignals.setConnected(inputs.indexerIntakeMotorConnected);
        pivotSignals.setConnected(inputs.pivotMotorConnected);

        inputs.frontRollerMotorInputs = frontRollerSignals.getMotorInputs();
        inputs.indexerMotorInputs = indexerSignals.getMotorInputs();
        inputs.pivotMotorInputs = pivotSignals.getMotorInputs();

        inputs.beambreak = beambreak.get();
    }

    @Override
    public void setFrontRollerSpeed(double volts) {
        talonFrontRoller.setVoltage(volts);
    }

    @Override
    public void setIndexerSpeed(double volts) {
        talonIndexer.setVoltage(volts);
    }

    @Override
    public void setPivotTargetAngle(Angle angle) {
        talonPivot.setPosition(angle);
    }
}
