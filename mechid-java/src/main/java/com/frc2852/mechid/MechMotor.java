package com.frc2852.mechid;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Vendor abstraction over the motor controllers MechID can characterize.
 * Implementations: {@link TalonFxMotor} (CTRE Kraken X60/X44) and
 * {@link SparkFlexMotor} (REV Vortex).
 *
 * <p>All positions/velocities are mechanism units — whatever the controller reports
 * after its configured conversion (SensorToMechanismRatio / conversion factors).
 */
interface MechMotor {
    String vendorName();

    // --- Drive ---
    void applyVolts(double volts);

    /** Stop respecting the configured neutral/idle mode. */
    void stopNeutral();

    /** Stop with the rotor free-spinning if the vendor supports it; else neutral. */
    void stopCoast();

    /** True if {@link #stopCoast()} genuinely coasts regardless of the neutral config. */
    boolean supportsCoast();

    void setFoc(boolean foc);

    // --- Telemetry (latest values; bulk refresh handled via bulkRefreshSignals) ---
    double position();

    double velocity();

    /** Onboard filtered acceleration, or NaN if the vendor provides none. */
    double acceleration();

    double appliedVolts();

    double statorCurrent();

    double supplyCurrent();

    /** Signals to include in the recorder's single bulk refresh (CTRE only; may be empty). */
    BaseStatusSignal[] bulkRefreshSignals();

    // --- Routine lifecycle ---
    void beginHighRateSampling(double hz);

    void endHighRateSampling();

    void saveAndDisableSoftLimits();

    void restoreSoftLimits();

    /** Snapshot the active control request so it can be restored after the routine. */
    void saveAppliedControl();

    void restoreAppliedControl();

    // --- Device description ---
    /** Rotor rotations per mechanism unit, derived from the device configuration. */
    double rotorToMechanismRatio();

    /** False when the device config cannot reveal the true gearing (e.g. RemoteCANcoder). */
    boolean ratioTrusted();

    /** Append this device's as-configured state to meta.json (called with key already set). */
    void writeDeviceMeta(JsonBuilder json);
}
