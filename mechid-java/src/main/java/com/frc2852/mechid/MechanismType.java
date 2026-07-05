package com.frc2852.mechid;

/**
 * The kinds of FRC mechanisms MechID knows how to characterize. The type determines
 * which feedforward terms are identified, whether the routine must guard position
 * limits, and whether the resulting PID gains are for a position or velocity loop.
 */
public enum MechanismType {
    /** Position-controlled linear mechanism working against constant gravity (kG). */
    ELEVATOR(PidDomain.POSITION, GravityModel.CONSTANT, true),

    /** Position-controlled rotating mechanism with no gravity load and limited travel. */
    TURRET(PidDomain.POSITION, GravityModel.NONE, true),

    /** Position-controlled pivoting arm; gravity torque varies with cos(angle). */
    ARM(PidDomain.POSITION, GravityModel.COSINE, true),

    /** Velocity-controlled rotating mechanism (flywheel, roller, indexer wheel). */
    FLYWHEEL(PidDomain.VELOCITY, GravityModel.NONE, false),

    /** Swerve drive motors (velocity loop). Characterized via {@link SwerveMechId}. */
    SWERVE_DRIVE(PidDomain.VELOCITY, GravityModel.NONE, true),

    /** Swerve azimuth/steer motors (position loop, continuous rotation). Characterized via {@link SwerveMechId}. */
    SWERVE_STEER(PidDomain.POSITION, GravityModel.NONE, false);

    /** Which closed-loop domain the mechanism is ultimately controlled in. */
    public enum PidDomain {
        POSITION,
        VELOCITY
    }

    /** How gravity loads the mechanism, matching Phoenix 6 {@code GravityTypeValue} semantics. */
    public enum GravityModel {
        /** No gravity term (turret, flywheel, swerve). */
        NONE,
        /** Constant gravity force regardless of position (elevator) — Phoenix {@code Elevator_Static}. */
        CONSTANT,
        /** Gravity torque proportional to cos(position), position 0 = horizontal — Phoenix {@code Arm_Cosine}. */
        COSINE;
    }

    private final PidDomain pidDomain;
    private final GravityModel gravityModel;
    private final boolean travelLimited;

    MechanismType(PidDomain pidDomain, GravityModel gravityModel, boolean travelLimited) {
        this.pidDomain = pidDomain;
        this.gravityModel = gravityModel;
        this.travelLimited = travelLimited;
    }

    public PidDomain pidDomain() {
        return pidDomain;
    }

    public GravityModel gravityModel() {
        return gravityModel;
    }

    /** True if the test routine must guard against running past position limits. */
    public boolean isTravelLimited() {
        return travelLimited;
    }
}
