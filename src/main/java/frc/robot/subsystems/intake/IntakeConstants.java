package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.*;

public class IntakeConstants {
    public static final CANBus intakeCANBus = new CANBus("rio");

    public static final int indexerMotorId = 31;
    public static final int pivotMotorId = 32;
    public static final int frontRollerMotorId = 33;

    public static final int beambreakPort = 1;

    public static final double intakeingVoltage = 12;
    public static final double feedingVoltage = -12;

    public static final Angle pivotStowedAngle = Degrees.of(90); // TODO: make actual values

    public static final Angle pivotExtendedAngle = Degrees.of(0);

    public static final Angle pivotFeedingAngle = Degrees.of(160);

    public static final Angle pivotTolerance = Degrees.of(1);

    public static class motorConfigs {
        public static final TalonFXConfiguration indexerMotor = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30)
                        .withSupplyCurrentLimitEnable(true));

        public static final TalonFXConfiguration pivotMotor = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30)
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                        .withSensorToMechanismRatio(20))
                .withSlot0(new Slot0Configs()
                        .withKP(10)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0));

        public static final TalonFXConfiguration frontRollerMotor = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30)
                        .withSupplyCurrentLimitEnable(true));
    }
}
