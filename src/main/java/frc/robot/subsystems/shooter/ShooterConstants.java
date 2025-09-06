package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.*;

public class ShooterConstants {

    public static final int beambreakId = 0;

    public static final CANBus shooterCANBus = new CANBus("rio");

    public static final int bottomLeftShooterMotorId = 21;
    public static final int bottomRightShooterMotorId = 22;
    public static final int topShooterMotorId = 23;
    public static final int feederMotorId = 24;
    public static final int pivotLeftMotorId = 25;
    public static final int pivotRightMotorId = 26;

    public static final Angle pivotTolerance = Degrees.of(1);

    public static final AngularVelocity rollerSpeedTolerance = RotationsPerSecond.of(50);

    public static final double feederSpeedLoading = 5; // volts

    public static final double feederSpeedShooting = 12;

    public static final double shootingTime = 1; // Seconds

    public static final SlewRateLimiter bottomWheelLimiter = new SlewRateLimiter(6000); // Rps
    public static final SlewRateLimiter topWheelLimiter = new SlewRateLimiter(6000); // for sim

    public static enum Setpoints {
        Load(new Setpoint(Degrees.of(30), RotationsPerSecond.of(0), RotationsPerSecond.of(0))),
        L1(new Setpoint(Degrees.of(40), RotationsPerSecond.of(1000), RotationsPerSecond.of(1500))),
        L2(new Setpoint(Degrees.of(50), RotationsPerSecond.of(50), RotationsPerSecond.of(2000))),
        L3(new Setpoint(Degrees.of(60), RotationsPerSecond.of(1500), RotationsPerSecond.of(3000))),
        L4(new Setpoint(Degrees.of(70), RotationsPerSecond.of(2000), RotationsPerSecond.of(4000)));

        private final Setpoint setpoint;

        Setpoints(Setpoint setpoint) {
            this.setpoint = setpoint;
        }

        public Setpoint getSetpoint() {
            return setpoint;
        }
    }

    public static record Setpoint(Angle pivotAngle, AngularVelocity bottomWheelSpeed, AngularVelocity topWheelSpeed) {}

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
