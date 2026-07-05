package com.frc2852.mechid;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Runs the full identification sequence for one mechanism and records the data:
 *
 * <ol>
 *   <li>Quasistatic voltage ramp, forward (and reverse for travel-limited mechanisms)
 *       — identifies kS, kV, and kG</li>
 *   <li>Dynamic voltage steps at several amplitudes — identifies kA and validates
 *       linearity across the current range</li>
 *   <li>Coast-down (velocity mechanisms) — isolates friction from back-EMF</li>
 * </ol>
 *
 * <p>Every phase is guarded: travel-limited mechanisms reverse with margin before their
 * limits, and the routine hard-aborts on over-velocity, sustained over-current, position
 * escape, or timeout. Partial data is still flushed to disk on abort.
 */
class MechIdRoutineCommand extends Command {

    private final MechIdConfig config;
    private final RoutineIo io;
    private final List<SignalRecorder.Channel> channels;
    private final Map<String, Double> extraMeta;
    private final String dashboardPrefix;

    private SignalRecorder recorder;
    private SessionWriter session;

    private final List<Phase> phases = new ArrayList<>();
    private int phaseIndex;
    private double routineStartTime;
    private double statorViolationStart = Double.NaN;
    private boolean aborted;
    private boolean ioStarted;

    MechIdRoutineCommand(MechIdConfig config, Subsystem requirement, RoutineIo io,
            List<SignalRecorder.Channel> channels, Map<String, Double> extraMeta) {
        this.config = config;
        this.io = io;
        this.channels = channels;
        this.extraMeta = extraMeta;
        this.dashboardPrefix = "MechID/" + config.name() + "/";
        setName("MechID-" + config.name());
        addRequirements(requirement);
    }

    // ------------------------------------------------------------------
    // Phases
    // ------------------------------------------------------------------

    private abstract class Phase {
        String id;
        double startTime;

        Phase(String id) {
            this.id = id;
        }

        final void begin(double now) {
            startTime = now;
            onBegin();
            publish("Phase: " + id);
        }

        void onBegin() {
        }

        /** Advance one cycle; return true when the phase is complete. */
        abstract boolean run(double elapsed);

        /** +1/-1 while commanding motion (enables the travel guard), 0 otherwise. */
        int direction() {
            return 0;
        }
    }

    /** Slow voltage ramp for quasistatic (steady-state) data. */
    private class RampPhase extends Phase {
        private final int dir;

        RampPhase(int dir) {
            super(dir > 0 ? "quasistatic_forward" : "quasistatic_reverse");
            this.dir = dir;
        }

        @Override
        boolean run(double elapsed) {
            double volts = dir * Math.min(elapsed * config.quasistaticRampRate(), config.quasistaticMaxVoltage());
            apply(volts);
            boolean voltageCapped = Math.abs(volts) >= config.quasistaticMaxVoltage();
            boolean nearVelocityLimit = !Double.isNaN(config.maxVelocityAbort())
                    && Math.abs(io.guardVelocity()) > 0.7 * config.maxVelocityAbort();
            return voltageCapped || nearVelocityLimit;
        }

        @Override
        int direction() {
            return dir;
        }
    }

    /** Constant-voltage step for dynamic (acceleration) data. */
    private class StepPhase extends Phase {
        private final double volts;
        private final boolean autoDirection;
        private int dir;

        StepPhase(String id, double volts, boolean autoDirection, int fixedDir) {
            super(id);
            this.volts = volts;
            this.autoDirection = autoDirection;
            this.dir = fixedDir;
        }

        @Override
        void onBegin() {
            if (autoDirection && config.type().isTravelLimited()) {
                double center = (config.minPosition() + config.maxPosition()) / 2.0;
                dir = io.guardPosition() < center ? 1 : -1;
                id = id + (dir > 0 ? "_fwd" : "_rev");
            }
        }

        @Override
        boolean run(double elapsed) {
            apply(dir * volts);
            return elapsed >= config.dynamicStepHoldSec();
        }

        @Override
        int direction() {
            return dir;
        }
    }

    /** Wait (neutral output) until the mechanism is still. */
    private class SettlePhase extends Phase {
        SettlePhase() {
            super("settle");
        }

        @Override
        boolean run(double elapsed) {
            stopNeutral();
            boolean still = Math.abs(io.guardVelocity()) < config.settleVelocityTolerance();
            return (still && elapsed >= config.settleTimeSec()) || elapsed >= config.settleTimeoutSec();
        }
    }

    /** Spin up then cut to coast; the decay isolates friction from back-EMF. */
    private class SpinupPhase extends Phase {
        SpinupPhase() {
            super("coastdown_spinup");
        }

        @Override
        boolean run(double elapsed) {
            apply(config.coastdownSpinupVoltage());
            return elapsed >= config.coastdownSpinupSec();
        }

        @Override
        int direction() {
            return 1;
        }
    }

    private class CoastPhase extends Phase {
        CoastPhase() {
            super("coastdown");
        }

        @Override
        boolean run(double elapsed) {
            stopCoast();
            return Math.abs(io.guardVelocity()) < config.settleVelocityTolerance()
                    || elapsed >= config.coastdownTimeoutSec();
        }
    }

    private void buildPhases() {
        phases.clear();
        // Continuous-rotation position mechanisms (swerve steer) get bidirectional
        // excitation too — friction data from both directions, no travel to worry about.
        boolean bidirectional = config.type().isTravelLimited()
                || config.type() == MechanismType.SWERVE_STEER;

        phases.add(new RampPhase(1));
        phases.add(new SettlePhase());
        if (bidirectional) {
            phases.add(new RampPhase(-1));
            phases.add(new SettlePhase());
        }

        double[] steps = config.dynamicStepVoltages();
        for (int i = 0; i < steps.length; i++) {
            String id = String.format("dynamic_s%d_%.1fV", i, steps[i]);
            if (config.type().isTravelLimited()) {
                // Two pulses per amplitude; direction resolved at runtime toward open travel,
                // so the mechanism bounces between its limits.
                phases.add(new StepPhase(id + "_a", steps[i], true, 1));
                phases.add(new SettlePhase());
                phases.add(new StepPhase(id + "_b", steps[i], true, 1));
            } else if (bidirectional) {
                phases.add(new StepPhase(id + "_fwd", steps[i], false, 1));
                phases.add(new SettlePhase());
                phases.add(new StepPhase(id + "_rev", steps[i], false, -1));
            } else {
                phases.add(new StepPhase(id, steps[i], false, 1));
            }
            phases.add(new SettlePhase());
        }

        if (config.coastdownEnabled()) {
            phases.add(new SpinupPhase());
            phases.add(new CoastPhase());
            phases.add(new SettlePhase());
        }
    }

    // ------------------------------------------------------------------
    // Command lifecycle
    // ------------------------------------------------------------------

    @Override
    public void initialize() {
        aborted = false;
        phaseIndex = 0;
        statorViolationStart = Double.NaN;
        routineStartTime = Timer.getFPGATimestamp();

        try {
            config.validate();
        } catch (IllegalStateException e) {
            DriverStation.reportError("[MechID] " + e.getMessage(), false);
            aborted = true;
            return;
        }

        io.onRoutineStart();
        ioStarted = true;

        try {
            session = new SessionWriter(config);
            List<String> names = new ArrayList<>();
            List<MechMotor> motors = new ArrayList<>();
            List<String> roles = new ArrayList<>();
            for (SignalRecorder.Channel ch : channels) {
                names.add(ch.name);
                motors.add(ch.motor);
                roles.add(ch.role);
            }
            session.writeMeta(names, motors, roles, extraMeta);
            recorder = new SignalRecorder(channels, config.sampleRateHz());
            recorder.start(session.dataCsv());
        } catch (IOException e) {
            DriverStation.reportError("[MechID] Failed to open session: " + e.getMessage(), false);
            aborted = true;
            return;
        }

        buildPhases();
        phases.get(0).begin(routineStartTime);
        SmartDashboard.putString(dashboardPrefix + "Session", session.sessionDir().toString());
        System.out.println("[MechID] " + config.name() + ": routine started, "
                + phases.size() + " phases -> " + session.sessionDir());
    }

    @Override
    public void execute() {
        if (aborted || phaseIndex >= phases.size()) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (!checkSafety(now)) {
            return;
        }

        Phase phase = phases.get(phaseIndex);
        boolean complete = phase.run(now - phase.startTime);

        // Travel guard: end the driven phase with margin to spare before the limit.
        int dir = phase.direction();
        if (!complete && dir != 0 && config.type().isTravelLimited()) {
            double pos = io.guardPosition();
            if ((dir > 0 && pos >= config.maxPosition() - config.positionMargin())
                    || (dir < 0 && pos <= config.minPosition() + config.positionMargin())) {
                complete = true;
            }
        }

        if (complete) {
            phaseIndex++;
            if (phaseIndex < phases.size()) {
                phases.get(phaseIndex).begin(now);
            }
        }
    }

    private boolean checkSafety(double now) {
        if (now - routineStartTime > config.routineTimeoutSec()) {
            abort("routine timeout after " + config.routineTimeoutSec() + "s");
            return false;
        }

        if (!Double.isNaN(config.maxVelocityAbort())
                && Math.abs(io.guardVelocity()) > config.maxVelocityAbort()) {
            abort(String.format("velocity %.2f exceeded abort limit %.2f",
                    io.guardVelocity(), config.maxVelocityAbort()));
            return false;
        }

        if (config.type().isTravelLimited()) {
            double pos = io.guardPosition();
            double grace = 0.5 * config.positionMargin();
            if (pos > config.maxPosition() + grace || pos < config.minPosition() - grace) {
                abort(String.format("position %.3f escaped range [%.3f, %.3f]",
                        pos, config.minPosition(), config.maxPosition()));
                return false;
            }
        }

        double stator = io.maxStatorCurrent();
        if (stator > config.statorCurrentAbort()) {
            if (Double.isNaN(statorViolationStart)) {
                statorViolationStart = now;
            } else if (now - statorViolationStart > config.statorAbortDebounceSec()) {
                abort(String.format("stator current %.0fA exceeded %.0fA for %.2fs",
                        stator, config.statorCurrentAbort(), config.statorAbortDebounceSec()));
                return false;
            }
        } else {
            statorViolationStart = Double.NaN;
        }
        return true;
    }

    private void abort(String reason) {
        aborted = true;
        DriverStation.reportError("[MechID] " + config.name() + " ABORTED: " + reason, false);
        publish("ABORTED: " + reason);
        cancel();
    }

    @Override
    public boolean isFinished() {
        return aborted || phaseIndex >= phases.size();
    }

    @Override
    public void end(boolean interrupted) {
        if (ioStarted) {
            io.stopNeutral();
        }
        if (recorder != null) {
            recorder.close();
            recorder = null;
        }
        if (ioStarted) {
            io.onRoutineEnd();
            ioStarted = false;
        }

        String outcome = aborted ? "ABORTED (partial data saved)"
                : interrupted ? "CANCELLED (partial data saved)" : "COMPLETE";
        publish(outcome);
        if (session != null) {
            System.out.println("[MechID] " + config.name() + ": " + outcome
                    + " -> " + session.sessionDir());
            System.out.println("[MechID] Copy to your computer and run: mechid analyze "
                    + session.sessionDir().getFileName());
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    private void apply(double volts) {
        io.applyVolts(volts);
        if (recorder != null) {
            recorder.setContext(currentPhaseId(), volts);
        }
    }

    private void stopNeutral() {
        io.stopNeutral();
        if (recorder != null) {
            recorder.setContext(currentPhaseId(), 0.0);
        }
    }

    private void stopCoast() {
        io.stopCoast();
        if (recorder != null) {
            recorder.setContext(currentPhaseId(), 0.0);
        }
    }

    private String currentPhaseId() {
        return phaseIndex < phases.size() ? phases.get(phaseIndex).id : "done";
    }

    private void publish(String status) {
        SmartDashboard.putString(dashboardPrefix + "Status", status);
    }
}
