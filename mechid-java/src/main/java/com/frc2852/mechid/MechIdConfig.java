package com.frc2852.mechid;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Describes one mechanism to MechID: which motors drive it, what kind of mechanism it
 * is, its safe operating envelope, and how aggressively the identification routine may
 * excite it.
 *
 * <p>Create one via the static factory for your mechanism type (overloads exist for
 * CTRE {@link TalonFX} and REV {@link SparkFlex}), then chain {@code with...} calls:
 *
 * <pre>{@code
 * MechIdConfig config = MechIdConfig.turret("turret", motor, MotorModel.KRAKEN_X60)
 *         .withPositionRange(-0.51, 0.51)          // mechanism rotations
 *         .withRotorToMechanismRatio(50.0)          // required if using RemoteCANcoder feedback
 *         .withMaxVelocityAbort(3.0);               // rot/s
 * }</pre>
 *
 * <h2>Multi-motor mechanisms</h2>
 * Two ways to describe the other motors, chosen by how they are controlled:
 * <ul>
 *   <li>{@link #withFollowers}: motors already configured with vendor follower control
 *       (Phoenix {@code Follower} request, REV {@code config.follow()}). They mirror
 *       the leader's output on their own, so MechID records them but never commands
 *       them.</li>
 *   <li>{@link #withSynchronizedMotors}: independent motors on the same mechanism that
 *       are NOT followers. MechID applies the identical voltage to each of them and the
 *       leader simultaneously (each motor's own inversion config must make +V =
 *       mechanism-forward), and restores their previous control afterward.</li>
 * </ul>
 *
 * <p>All positions and velocities are in <b>mechanism units</b> — whatever the motor
 * controller reports after its configured conversion. Identified gains come back in
 * the same units, ready to paste into {@code Slot0Configs}.
 */
public class MechIdConfig {

    private final String name;
    private final MechanismType type;
    private final MotorModel motorModel;
    private final MechMotor leader;
    private final List<MechMotor> followers = new ArrayList<>();
    private final List<MechMotor> synchronizedMotors = new ArrayList<>();

    // Travel envelope (mechanism units). NaN = not applicable / not set.
    private double minPosition = Double.NaN;
    private double maxPosition = Double.NaN;
    private double positionMarginFraction = 0.08;

    // Safety aborts
    private double maxVelocityAbort = Double.NaN; // mechanism units/s; NaN disables
    private double statorCurrentAbort;
    private double statorAbortDebounceSec = 0.10;
    private double routineTimeoutSec = 240.0;

    // Quasistatic ramp
    private double quasistaticRampRate; // V/s
    private double quasistaticMaxVoltage;

    // Dynamic steps
    private double[] dynamicStepVoltages;
    private double dynamicStepHoldSec;

    // Settle between phases
    private double settleTimeSec = 1.0;
    private double settleVelocityTolerance = 0.05;
    private double settleTimeoutSec = 8.0;

    // Coast-down (velocity mechanisms)
    private boolean coastdownEnabled;
    private double coastdownSpinupVoltage = 8.0;
    private double coastdownSpinupSec = 2.0;
    private double coastdownTimeoutSec = 30.0;

    private double sampleRateHz = 200.0;
    private boolean useFoc = false;
    private boolean manageSoftLimits = true;
    private double rotorToMechanismRatio = Double.NaN; // NaN = read from device config
    private double armHorizontalPosition = 0.0;
    private String positionUnits = "rot";
    private String outputRootOverride = null;

    private MechIdConfig(String name, MechanismType type, MechMotor leader, MotorModel motorModel) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("Mechanism name must not be blank");
        }
        if (leader == null) {
            throw new IllegalArgumentException("Leader motor must not be null");
        }
        this.name = name.replaceAll("[^A-Za-z0-9_-]", "_");
        this.type = type;
        this.leader = leader;
        this.motorModel = motorModel;
    }

    // ---------------------------------------------------------------------
    // Factories with per-type excitation defaults
    // ---------------------------------------------------------------------

    /** Position-controlled elevator (TalonFX). Set the travel range with {@link #withPositionRange}. */
    public static MechIdConfig elevator(String name, TalonFX leader, MotorModel motorModel) {
        return elevator(name, new TalonFxMotor(leader), motorModel);
    }

    /** Position-controlled elevator (SparkFlex). Set the travel range with {@link #withPositionRange}. */
    public static MechIdConfig elevator(String name, SparkFlex leader, MotorModel motorModel) {
        return elevator(name, new SparkFlexMotor(leader), motorModel);
    }

    private static MechIdConfig elevator(String name, MechMotor leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.ELEVATOR, leader, motorModel);
        c.quasistaticRampRate = 0.35;
        c.quasistaticMaxVoltage = 5.0;
        c.dynamicStepVoltages = new double[] { 2.0, 3.0, 4.0 };
        c.dynamicStepHoldSec = 1.0;
        c.statorCurrentAbort = 80.0;
        c.coastdownEnabled = false;
        return c;
    }

    /** Position-controlled turret (TalonFX). Set the travel range with {@link #withPositionRange}. */
    public static MechIdConfig turret(String name, TalonFX leader, MotorModel motorModel) {
        return turret(name, new TalonFxMotor(leader), motorModel);
    }

    /** Position-controlled turret (SparkFlex). Set the travel range with {@link #withPositionRange}. */
    public static MechIdConfig turret(String name, SparkFlex leader, MotorModel motorModel) {
        return turret(name, new SparkFlexMotor(leader), motorModel);
    }

    private static MechIdConfig turret(String name, MechMotor leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.TURRET, leader, motorModel);
        c.quasistaticRampRate = 0.25;
        c.quasistaticMaxVoltage = 4.0;
        c.dynamicStepVoltages = new double[] { 1.5, 2.5, 3.5 };
        c.dynamicStepHoldSec = 1.5;
        c.statorCurrentAbort = 60.0;
        c.coastdownEnabled = false;
        return c;
    }

    /**
     * Position-controlled pivoting arm (TalonFX). Set the travel range with
     * {@link #withPositionRange}. MechID assumes Phoenix's {@code Arm_Cosine} convention
     * (position 0 = horizontal); if your zero is elsewhere, call
     * {@link #withArmHorizontalPosition}.
     */
    public static MechIdConfig arm(String name, TalonFX leader, MotorModel motorModel) {
        return arm(name, new TalonFxMotor(leader), motorModel);
    }

    /** Position-controlled pivoting arm (SparkFlex). See {@link #arm(String, TalonFX, MotorModel)}. */
    public static MechIdConfig arm(String name, SparkFlex leader, MotorModel motorModel) {
        return arm(name, new SparkFlexMotor(leader), motorModel);
    }

    private static MechIdConfig arm(String name, MechMotor leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.ARM, leader, motorModel);
        c.quasistaticRampRate = 0.25;
        c.quasistaticMaxVoltage = 4.0;
        c.dynamicStepVoltages = new double[] { 1.5, 2.5, 3.5 };
        c.dynamicStepHoldSec = 1.0;
        c.statorCurrentAbort = 60.0;
        c.coastdownEnabled = false;
        return c;
    }

    /** Velocity-controlled flywheel/roller (TalonFX). Spins in the positive direction only. */
    public static MechIdConfig flywheel(String name, TalonFX leader, MotorModel motorModel) {
        return flywheel(name, new TalonFxMotor(leader), motorModel);
    }

    /** Velocity-controlled flywheel/roller (SparkFlex). Spins in the positive direction only. */
    public static MechIdConfig flywheel(String name, SparkFlex leader, MotorModel motorModel) {
        return flywheel(name, new SparkFlexMotor(leader), motorModel);
    }

    private static MechIdConfig flywheel(String name, MechMotor leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.FLYWHEEL, leader, motorModel);
        c.quasistaticRampRate = 0.5;
        c.quasistaticMaxVoltage = 10.0;
        c.dynamicStepVoltages = new double[] { 2.0, 4.0, 6.0, 8.0, 10.0 };
        c.dynamicStepHoldSec = 3.0;
        c.statorCurrentAbort = 100.0;
        c.coastdownEnabled = true;
        c.settleVelocityTolerance = 0.5;
        return c;
    }

    // Package-private factories used by SwerveMechId (motors are recorded per module).
    static MechIdConfig swerveDrive(String name, TalonFX leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.SWERVE_DRIVE, new TalonFxMotor(leader), motorModel);
        c.quasistaticRampRate = 1.0;
        c.quasistaticMaxVoltage = 8.0;
        c.dynamicStepVoltages = new double[] { 3.0, 5.0, 7.0 };
        c.dynamicStepHoldSec = 1.5;
        c.statorCurrentAbort = 120.0;
        c.coastdownEnabled = false;
        return c;
    }

    static MechIdConfig swerveSteer(String name, TalonFX leader, MotorModel motorModel) {
        MechIdConfig c = new MechIdConfig(name, MechanismType.SWERVE_STEER, new TalonFxMotor(leader), motorModel);
        c.quasistaticRampRate = 0.5;
        c.quasistaticMaxVoltage = 6.0;
        c.dynamicStepVoltages = new double[] { 2.0, 3.0, 4.0 };
        c.dynamicStepHoldSec = 1.0;
        c.statorCurrentAbort = 60.0;
        c.coastdownEnabled = false;
        return c;
    }

    // ---------------------------------------------------------------------
    // Builder-style setters
    // ---------------------------------------------------------------------

    /**
     * Motors already configured with vendor follower control (Phoenix {@code Follower},
     * REV {@code config.follow()}). They mirror the leader on their own, so MechID
     * records them for the current budget but never commands them. For non-follower
     * motors on the same mechanism use {@link #withSynchronizedMotors}.
     */
    public MechIdConfig withFollowers(TalonFX... followerMotors) {
        for (TalonFX m : followerMotors) {
            followers.add(new TalonFxMotor(m));
        }
        return this;
    }

    /** See {@link #withFollowers(TalonFX...)}. */
    public MechIdConfig withFollowers(SparkFlex... followerMotors) {
        for (SparkFlex m : followerMotors) {
            followers.add(new SparkFlexMotor(m));
        }
        return this;
    }

    /**
     * Independent motors on the same mechanism that are NOT configured as followers.
     * MechID drives them with the identical voltage as the leader for the whole
     * routine — each motor's own inversion config must make +V = mechanism-forward —
     * and restores their previous control request when the routine ends.
     */
    public MechIdConfig withSynchronizedMotors(TalonFX... motors) {
        for (TalonFX m : motors) {
            synchronizedMotors.add(new TalonFxMotor(m));
        }
        return this;
    }

    /** See {@link #withSynchronizedMotors(TalonFX...)}. */
    public MechIdConfig withSynchronizedMotors(SparkFlex... motors) {
        for (SparkFlex m : motors) {
            synchronizedMotors.add(new SparkFlexMotor(m));
        }
        return this;
    }

    /**
     * Safe travel range in mechanism units. Required for elevators, turrets, and arms.
     * The routine reverses direction with a margin before these limits and hard-aborts
     * if the mechanism ever moves past them.
     */
    public MechIdConfig withPositionRange(double min, double max) {
        if (max <= min) {
            throw new IllegalArgumentException("maxPosition must be greater than minPosition");
        }
        this.minPosition = min;
        this.maxPosition = max;
        return this;
    }

    /** Fraction of the travel range kept as a buffer before each limit (default 0.08). */
    public MechIdConfig withPositionMarginFraction(double fraction) {
        this.positionMarginFraction = fraction;
        return this;
    }

    /** Abort the routine if |velocity| exceeds this (mechanism units/s). Strongly recommended. */
    public MechIdConfig withMaxVelocityAbort(double maxVelocity) {
        this.maxVelocityAbort = maxVelocity;
        return this;
    }

    /** Abort if stator current exceeds this for longer than the debounce time (default per type). */
    public MechIdConfig withStatorCurrentAbort(double amps) {
        this.statorCurrentAbort = amps;
        return this;
    }

    /** Quasistatic ramp rate in volts/second (default per type). */
    public MechIdConfig withQuasistaticRamp(double voltsPerSecond, double maxVoltage) {
        this.quasistaticRampRate = voltsPerSecond;
        this.quasistaticMaxVoltage = maxVoltage;
        return this;
    }

    /** Voltages for the dynamic step tests and how long each step is held. */
    public MechIdConfig withDynamicSteps(double[] voltages, double holdSeconds) {
        this.dynamicStepVoltages = voltages.clone();
        this.dynamicStepHoldSec = holdSeconds;
        return this;
    }

    /**
     * Enable/disable the coast-down test (velocity mechanisms only; default on for
     * TalonFX flywheels). Unavailable for SparkFlex leaders — REV cannot force coast
     * mode from a control request.
     */
    public MechIdConfig withCoastdown(boolean enabled) {
        this.coastdownEnabled = enabled;
        return this;
    }

    /** Data sample rate in Hz (default 200, clamped to [50, 500]). */
    public MechIdConfig withSampleRate(double hz) {
        this.sampleRateHz = Math.max(50.0, Math.min(500.0, hz));
        return this;
    }

    /** Use FOC on the voltage requests (TalonFX with Phoenix Pro only). Default false. */
    public MechIdConfig withFoc(boolean foc) {
        this.useFoc = foc;
        return this;
    }

    /**
     * Whether MechID temporarily disables software limit switches on the commanded
     * motors during the routine and restores them afterward (default true). MechID
     * enforces its own limits from {@link #withPositionRange}.
     */
    public MechIdConfig withSoftLimitManagement(boolean manage) {
        this.manageSoftLimits = manage;
        return this;
    }

    /**
     * Total gearing between the motor rotor and the mechanism (rotor rotations per
     * mechanism unit). Normally read from the device configuration, but that is wrong
     * when a TalonFX uses {@code RemoteCANcoder} feedback — set it explicitly in that
     * case so the current-limit analysis knows the true rotor speed.
     */
    public MechIdConfig withRotorToMechanismRatio(double ratio) {
        this.rotorToMechanismRatio = ratio;
        return this;
    }

    /** For arms whose reported position is not 0 when horizontal: the position (mechanism units) at horizontal. */
    public MechIdConfig withArmHorizontalPosition(double position) {
        this.armHorizontalPosition = position;
        return this;
    }

    /** Label for position units in reports, e.g. "rot" or "m" (default "rot"). */
    public MechIdConfig withPositionUnits(String units) {
        this.positionUnits = units;
        return this;
    }

    /** Override where session folders are written (default {@code <operating dir>/mechid}). */
    public MechIdConfig withOutputRoot(String path) {
        this.outputRootOverride = path;
        return this;
    }

    /** Hard cap on total routine runtime in seconds (default 240). */
    public MechIdConfig withRoutineTimeout(double seconds) {
        this.routineTimeoutSec = seconds;
        return this;
    }

    /** How long the mechanism must be still between phases, and the velocity considered "still". */
    public MechIdConfig withSettle(double settleSeconds, double velocityTolerance) {
        this.settleTimeSec = settleSeconds;
        this.settleVelocityTolerance = velocityTolerance;
        return this;
    }

    // ---------------------------------------------------------------------
    // Validation
    // ---------------------------------------------------------------------

    /** Throws if the configuration is unusable; downgrades options the hardware can't honor. */
    void validate() {
        if (type.isTravelLimited() && type != MechanismType.SWERVE_DRIVE
                && (Double.isNaN(minPosition) || Double.isNaN(maxPosition))) {
            throw new IllegalStateException(
                    "MechID '" + name + "': " + type + " requires withPositionRange(min, max)");
        }
        if (dynamicStepVoltages == null || dynamicStepVoltages.length == 0) {
            throw new IllegalStateException("MechID '" + name + "': at least one dynamic step voltage required");
        }
        if (coastdownEnabled && !leader.supportsCoast()) {
            coastdownEnabled = false;
            DriverStation.reportWarning("[MechID] '" + name + "': coast-down test disabled — "
                    + leader.vendorName() + " cannot force coast mode", false);
        }
    }

    // ---------------------------------------------------------------------
    // Accessors (package-private where only the library needs them)
    // ---------------------------------------------------------------------

    public String name() {
        return name;
    }

    public MechanismType type() {
        return type;
    }

    public MotorModel motorModel() {
        return motorModel;
    }

    MechMotor leader() {
        return leader;
    }

    List<MechMotor> followers() {
        return Collections.unmodifiableList(followers);
    }

    List<MechMotor> synchronizedMotors() {
        return Collections.unmodifiableList(synchronizedMotors);
    }

    /** Leader plus synchronized motors — everything MechID commands directly. */
    List<MechMotor> commandedMotors() {
        List<MechMotor> all = new ArrayList<>();
        all.add(leader);
        all.addAll(synchronizedMotors);
        return all;
    }

    /** Every motor on the mechanism, in recording order: leader, synchronized, followers. */
    List<MechMotor> allMotors() {
        List<MechMotor> all = commandedMotors();
        all.addAll(followers);
        return all;
    }

    double minPosition() {
        return minPosition;
    }

    double maxPosition() {
        return maxPosition;
    }

    /** Absolute margin (mechanism units) kept before each position limit. */
    double positionMargin() {
        return (maxPosition - minPosition) * positionMarginFraction;
    }

    double maxVelocityAbort() {
        return maxVelocityAbort;
    }

    double statorCurrentAbort() {
        return statorCurrentAbort;
    }

    double statorAbortDebounceSec() {
        return statorAbortDebounceSec;
    }

    double routineTimeoutSec() {
        return routineTimeoutSec;
    }

    double quasistaticRampRate() {
        return quasistaticRampRate;
    }

    double quasistaticMaxVoltage() {
        return quasistaticMaxVoltage;
    }

    double[] dynamicStepVoltages() {
        return dynamicStepVoltages;
    }

    double dynamicStepHoldSec() {
        return dynamicStepHoldSec;
    }

    double settleTimeSec() {
        return settleTimeSec;
    }

    double settleVelocityTolerance() {
        return settleVelocityTolerance;
    }

    double settleTimeoutSec() {
        return settleTimeoutSec;
    }

    boolean coastdownEnabled() {
        return coastdownEnabled;
    }

    double coastdownSpinupVoltage() {
        return coastdownSpinupVoltage;
    }

    double coastdownSpinupSec() {
        return coastdownSpinupSec;
    }

    double coastdownTimeoutSec() {
        return coastdownTimeoutSec;
    }

    double sampleRateHz() {
        return sampleRateHz;
    }

    boolean useFoc() {
        return useFoc;
    }

    boolean manageSoftLimits() {
        return manageSoftLimits;
    }

    double rotorToMechanismRatio() {
        return rotorToMechanismRatio;
    }

    double armHorizontalPosition() {
        return armHorizontalPosition;
    }

    String positionUnits() {
        return positionUnits;
    }

    String outputRootOverride() {
        return outputRootOverride;
    }
}
