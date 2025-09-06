package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.TalonFXUtil.MotorInputs;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {

    private double frontRollerSpeed = 0.0;
    private double indexerSpeed = 0.0;

    private ProfiledPIDController pivotDummyController = new ProfiledPIDController(2, 0, 0, new Constraints(3, 3));

    private Angle currentAngle = Radians.zero();

    // maple sim intake
    private final IntakeSimulation intakeSimulation;

    public boolean obtainGamePiece() {
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Coral",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Width of the intake
                Meters.of(0.7), // TODO: make constants
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(0.2),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // run the fake PID controller to simulate the pivot motor and simulator
        pivotDummyController.calculate(currentAngle.in(Rotation));

        currentAngle = Rotations.of(pivotDummyController.getSetpoint().position);

        // Simulate motor inputs
        inputs.frontRollerMotorInputs = new MotorInputs(frontRollerSpeed, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        inputs.indexerMotorInputs = new MotorInputs(indexerSpeed, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        inputs.pivotMotorInputs = new MotorInputs(currentAngle.in(Radians), 0.0, 0.0, 0.0, 0.0, 0.0, false);

        // Simulate motor connections
        inputs.frontRollerIntakeMotorConnected = true;
        inputs.indexerIntakeMotorConnected = true;
        inputs.pivotMotorConnected = true;

        // maple sim periodic
        if (frontRollerSpeed != 0) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }

        // Simulate beam break sensor
        inputs.beambreak = intakeSimulation.getGamePiecesAmount() == 0;

        Logger.recordOutput("SimulatedPiecesInIntake", intakeSimulation.getGamePiecesAmount());
    }

    @Override
    public void setFrontRollerSpeed(double volts) {
        frontRollerSpeed = volts;
    }

    @Override
    public void setIndexerSpeed(double volts) {
        indexerSpeed = volts;
    }

    @Override
    public void setPivotTargetAngle(Angle angle) {
        pivotDummyController.reset(currentAngle.in(Rotation));
        pivotDummyController.setGoal(angle.in(Rotations));
    }
}
