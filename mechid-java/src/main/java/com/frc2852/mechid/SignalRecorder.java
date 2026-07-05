package com.frc2852.mechid;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Samples motor telemetry on a background {@link Notifier} thread at a fixed rate and
 * streams rows to a CSV file. Each motor's signal/frame rates are raised for the
 * duration of the recording and restored afterward. CTRE signals are refreshed in one
 * bulk call per tick; REV values update asynchronously and are simply read.
 *
 * <p>Columns: {@code t, test, cmd_v, battery_v} then per motor
 * {@code <name>_pos, <name>_vel, <name>_acc, <name>_volts, <name>_stator_a, <name>_supply_a}.
 * Motors without an acceleration signal record NaN; the analysis differentiates
 * velocity instead.
 */
class SignalRecorder {

    /** One motor's telemetry plus the column-name prefix and role it is recorded under. */
    static class Channel {
        final String name;
        final MechMotor motor;
        final String role;

        Channel(String name, MechMotor motor, String role) {
            this.name = name;
            this.motor = motor;
            this.role = role;
        }
    }

    private final List<Channel> channels;
    private final double sampleRateHz;
    private final BaseStatusSignal[] bulkSignals;

    private final Notifier notifier;
    private PrintWriter writer;
    private final Object writerLock = new Object();

    // Context stamped onto each row by the routine command. Volatile doubles are
    // written atomically, which is all the consistency the log needs.
    private volatile String testId = "idle";
    private volatile double commandedVolts = 0.0;
    private boolean refreshWarned = false;

    SignalRecorder(List<Channel> channels, double sampleRateHz) {
        this.channels = channels;
        this.sampleRateHz = sampleRateHz;

        List<BaseStatusSignal> signals = new ArrayList<>();
        for (Channel ch : channels) {
            for (BaseStatusSignal s : ch.motor.bulkRefreshSignals()) {
                signals.add(s);
            }
        }
        this.bulkSignals = signals.toArray(new BaseStatusSignal[0]);
        this.notifier = new Notifier(this::sample);
        this.notifier.setName("MechID-SignalRecorder");
    }

    void setContext(String testId, double commandedVolts) {
        this.testId = testId;
        this.commandedVolts = commandedVolts;
    }

    /** Raises telemetry rates, opens the CSV, and starts sampling. */
    void start(Path csvFile) throws IOException {
        for (Channel ch : channels) {
            ch.motor.beginHighRateSampling(sampleRateHz);
        }

        StringBuilder header = new StringBuilder("t,test,cmd_v,battery_v");
        for (Channel ch : channels) {
            for (String col : new String[] { "pos", "vel", "acc", "volts", "stator_a", "supply_a" }) {
                header.append(',').append(ch.name).append('_').append(col);
            }
        }

        synchronized (writerLock) {
            writer = new PrintWriter(new BufferedWriter(new FileWriter(csvFile.toFile()), 1 << 16));
            writer.println(header);
        }
        notifier.startPeriodic(1.0 / sampleRateHz);
    }

    /** Stops sampling, flushes the CSV, and restores original telemetry rates. */
    void stop() {
        notifier.stop();
        synchronized (writerLock) {
            if (writer != null) {
                writer.flush();
                writer.close();
                writer = null;
            }
        }
        for (Channel ch : channels) {
            ch.motor.endHighRateSampling();
        }
    }

    void close() {
        stop();
        notifier.close();
    }

    private void sample() {
        if (bulkSignals.length > 0) {
            var status = BaseStatusSignal.refreshAll(bulkSignals);
            if (!status.isOK() && !refreshWarned) {
                refreshWarned = true;
                DriverStation.reportWarning("[MechID] Signal refresh: " + status
                        + " (reported once; check CAN bus health)", false);
            }
        }

        StringBuilder row = new StringBuilder(256);
        row.append(Timer.getFPGATimestamp())
                .append(',').append(testId)
                .append(',').append(commandedVolts)
                .append(',').append(RobotController.getBatteryVoltage());
        for (Channel ch : channels) {
            MechMotor m = ch.motor;
            row.append(',').append(m.position())
                    .append(',').append(m.velocity())
                    .append(',').append(m.acceleration())
                    .append(',').append(m.appliedVolts())
                    .append(',').append(m.statorCurrent())
                    .append(',').append(m.supplyCurrent());
        }

        synchronized (writerLock) {
            if (writer != null) {
                writer.println(row);
            }
        }
    }
}
