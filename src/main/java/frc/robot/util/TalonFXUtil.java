package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;

public class TalonFXUtil {
    public static record MotorInputs(
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double supplyCurrentAmps,
            double torqueCurrentAmps,
            double tempCelsius,
            boolean tempFault) {}

    public static class MotorStatusSignalSet {
        final StatusSignal<Angle> positionRad;
        final StatusSignal<AngularVelocity> velocityRadPerSec;
        final StatusSignal<Voltage> leftAppliedVoltage;
        final StatusSignal<Current> supplyCurrent;
        final StatusSignal<Current> torqueCurrent;
        final StatusSignal<Temperature> tempCelsius;
        final StatusSignal<Boolean> tempFault;

        private Alert connectedAlert;

        public MotorStatusSignalSet(TalonFX motor, String motorName, String Subsystem) {
            connectedAlert = new Alert(
                    Subsystem,
                    motorName + "Motor ID: " + motor.getDeviceID() + "is disconnected on CAN BUS: "
                            + motor.getNetwork(),
                    Alert.AlertType.kError);
            connectedAlert.set(false);
            this.positionRad = motor.getPosition();
            this.velocityRadPerSec = motor.getVelocity();
            this.leftAppliedVoltage = motor.getMotorVoltage();
            this.supplyCurrent = motor.getSupplyCurrent();
            this.torqueCurrent = motor.getTorqueCurrent();
            this.tempCelsius = motor.getDeviceTemp();
            this.tempFault = motor.getFault_DeviceTemp();
        }

        public BaseStatusSignal[] getBaseStatusSignals() {
            return new BaseStatusSignal[] {
                positionRad, velocityRadPerSec, leftAppliedVoltage, supplyCurrent, torqueCurrent, tempCelsius, tempFault
            };
        }

        public void setConnected(boolean connected) {
            if (connected) {
                connectedAlert.set(false);
            } else {
                connectedAlert.set(true);
            }
        }

        public MotorInputs getMotorInputs() {
            return new MotorInputs(
                    positionRad.getValue().in(Radians),
                    velocityRadPerSec.getValue().in(DegreesPerSecond),
                    leftAppliedVoltage.getValue().in(Volts),
                    supplyCurrent.getValue().in(Amps),
                    torqueCurrent.getValue().in(Amps),
                    tempCelsius.getValue().in(Celsius),
                    tempFault.getValue());
        }
    }
}
