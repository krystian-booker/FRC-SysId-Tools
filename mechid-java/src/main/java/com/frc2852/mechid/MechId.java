package com.frc2852.mechid;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Entry point for characterizing a single mechanism. Construct one per mechanism
 * (typically inside its subsystem) and bind {@link #routine()} to a test-mode button.
 *
 * <pre>{@code
 * // In your subsystem:
 * private final MechId mechId = new MechId(
 *         MechIdConfig.flywheel("flywheel", leaderMotor, MotorModel.KRAKEN_X60)
 *                 .withFollowers(followerMotor)
 *                 .withMaxVelocityAbort(105.0),
 *         this);
 *
 * public Command mechIdRoutine() {
 *     return mechId.routine();
 * }
 * }</pre>
 *
 * <p>Data lands in {@code /home/lvuser/mechid/<name>_<timestamp>/} on the roboRIO.
 * Pull it to your computer and run the MechID analysis tool to get feedforward, PID,
 * Motion Magic, and current-limit recommendations.
 *
 * <p>If your subsystem extends {@link CharacterizableSubsystem}, this wiring is done
 * for you.
 */
public class MechId {
    private final MechIdConfig config;
    private final Subsystem requirement;

    public MechId(MechIdConfig config, Subsystem requirement) {
        config.validate();
        this.config = config;
        this.requirement = requirement;
    }

    public MechIdConfig config() {
        return config;
    }

    /**
     * The full identification routine: quasistatic ramps, dynamic steps, and (for
     * velocity mechanisms) a coast-down, with all safety guards active. Runs for
     * roughly 30–120 seconds depending on mechanism type. Cancel any time — partial
     * data is still saved.
     */
    public Command routine() {
        return new MechIdRoutineCommand(config, requirement, new MotorGroupIo(config),
                buildChannels(), Map.of());
    }

    private List<SignalRecorder.Channel> buildChannels() {
        List<SignalRecorder.Channel> channels = new ArrayList<>();
        channels.add(new SignalRecorder.Channel("leader", config.leader(), "leader"));
        int i = 1;
        for (MechMotor m : config.synchronizedMotors()) {
            channels.add(new SignalRecorder.Channel("sync" + i++, m, "synchronized"));
        }
        i = 1;
        for (MechMotor m : config.followers()) {
            channels.add(new SignalRecorder.Channel("follower" + i++, m, "follower"));
        }
        return channels;
    }
}
