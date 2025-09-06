package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.TalonFXUtil.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public MotorInputs frontRollerMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs indexerMotorInputs = new MotorInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        public MotorInputs pivotMotorInputs = new MotorInputs(0, 0, 0, 0, 0, 0, false);

        public boolean frontRollerIntakeMotorConnected = false;
        public boolean indexerIntakeMotorConnected = false;
        public boolean pivotMotorConnected = false;

        public boolean beambreak = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    ;

    public default void setFrontRollerSpeed(double volts) {}
    ;

    public default void setIndexerSpeed(double volts) {}
    ;

    public default void setPivotTargetAngle(Angle angle) {}
    ;
}
