package com.frc2852.mechid;

/**
 * How the routine command actually drives and observes a mechanism. Single-motor
 * mechanisms use {@link TalonFxIo}; swerve tests drive through CTRE
 * {@code SwerveRequest}s instead (see {@link SwerveMechId}).
 */
interface RoutineIo {
    /** Apply an open-loop voltage to the mechanism. */
    void applyVolts(double volts);

    /** Stop, respecting the configured neutral mode (brake for arms/elevators). */
    void stopNeutral();

    /** Stop with the rotor free-spinning (coast-down test). */
    void stopCoast();

    /** Position used for travel guards, in the units of the configured position range. */
    double guardPosition();

    /** Velocity used for the velocity abort, in the units of {@code withMaxVelocityAbort}. */
    double guardVelocity();

    /** Largest stator current magnitude across the monitored motors, for the current abort. */
    double maxStatorCurrent();

    /** Called once when the routine starts (capture reference pose, disable soft limits, ...). */
    default void onRoutineStart() {
    }

    /** Called once when the routine ends, even on abort (restore soft limits, ...). */
    default void onRoutineEnd() {
    }
}
