package frc.robot.util.Tuning;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunedWait {
    private Time time;

    private final Time defaultTime;

    private final LoggedNetworkNumber networkNumberSeconds;

    public TunedWait(Time defaultWait, String subsystem, String key) {
        this.defaultTime = defaultWait;
        this.time = defaultWait;
        this.networkNumberSeconds =
                new LoggedNetworkNumber("/Tuning/" + subsystem + "/" + key, defaultWait.in(Seconds));
    }

    public Command getWaitCommand() {
        return Commands.defer(
                () -> {
                    if (Constants.enableNTTuning) {
                        time = Seconds.of(networkNumberSeconds.get());
                    } else {
                        time = defaultTime;
                    }
                    return new WaitCommand(time);
                },
                Set.of());
    }
}
