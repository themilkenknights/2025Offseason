package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConstants {
    public static class motorConfigs {
        static TalonFXConfiguration shooterMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                .withSlot0(new Slot0Configs()
                        .withKP(.1)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0));

        TalonFXConfiguration feederMotor = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true));

        TalonFXConfiguration angleMotor = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(100.0))
                .withSlot0(new Slot0Configs()
                        .withKP(12)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(1)
                        .withMotionMagicAcceleration(1)
                        .withMotionMagicJerk(1));
    }
}
