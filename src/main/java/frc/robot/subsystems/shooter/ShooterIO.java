package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.*;
import frc.robot.util.TalonFXUtil.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public MotorInputs bottomLeftMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs bottomRightMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs topMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs feederMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        // Rotations after the gearing (ex the actual position of the shooter)
        public MotorInputs pivotLeftMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs pivotRightMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

        public boolean bottomLeftMotorConnected = false;
        public boolean bottomRightMotorConnected = false;
        public boolean topMotorConnected = false;
        public boolean feederMotorConnected = false;
        public boolean pivotLeftMotorConnected = false;
        public boolean pivotRightMotorConnected = false;

        public boolean beambreak = false;
    }

    /**
     * Sets the speed of the bottom shooter.
     *
     * @param speed The desired angular velocity for the bottom shooter wheels.
     */
    public default void setBottomShooterSpeed(AngularVelocity speed) {}
    ;

    /**
     * Sets the speed of the top shooter.
     *
     * @param speed The desired angular velocity for the top shooter wheel.
     */
    public default void setTopShooterSpeed(AngularVelocity speed) {}
    ;

    /**
     * Sets the speed of the feeder.
     *
     * @param speed The desired voltage for the feeder motor.
     */
    public default void setFeederSpeed(double volts) {}

    /**
     * Sets the target angle for the shooter to pivot to using the built-in motor controller's motion control.
     *
     * @param angle The desired target angle for the shooter.
     */
    public default void setTargetAngle(Angle angle) {}
    ;

    public default void updateInputs(ShooterIOInputs inputs) {}
    ;

    public static enum SimGoal {
        L2,
        L3,
        L4,
        NONE
    }

    public default void setSimGoal(SimGoal simGoal) {}
    ;
}
