package com.frc2852.mechid;

/**
 * Parameters for characterizing a CTRE swerve drivetrain with {@link SwerveMechId}.
 *
 * <p>MechID records all four modules and the analysis fits one shared gain set for the
 * drive motors and one for the steer motors (it also reports per-module spread so a
 * mechanically unhealthy module stands out). Per-module gains are not worth the
 * calibration burden — modules are nominally identical, and CTRE's swerve API applies
 * one gain set to all modules anyway.
 */
public class SwerveIdConfig {
    private final MotorModel driveMotorModel;
    private final MotorModel steerMotorModel;
    private double travelMeters = 4.0;
    private double wheelRadiusMeters = Double.NaN;
    private double maxSpeedAbortMps = 4.5;
    private double maxSteerVelocityAbort = Double.NaN;

    /**
     * @param driveMotorModel motor used for the drive wheels (e.g. {@code KRAKEN_X60})
     * @param steerMotorModel motor used for azimuth steering (e.g. {@code KRAKEN_X44})
     */
    public SwerveIdConfig(MotorModel driveMotorModel, MotorModel steerMotorModel) {
        this.driveMotorModel = driveMotorModel;
        this.steerMotorModel = steerMotorModel;
    }

    /**
     * Free floor space in front of the robot for the drive test, in meters (default 4).
     * The routine drives forward up to this distance, then back toward the start.
     */
    public SwerveIdConfig withTravelMeters(double meters) {
        if (meters <= 0.5) {
            throw new IllegalArgumentException("Travel must be more than 0.5 m");
        }
        this.travelMeters = meters;
        return this;
    }

    /** Wheel radius in meters (from TunerConstants.kWheelRadius). Lets the analysis report drive gains in m/s. */
    public SwerveIdConfig withWheelRadius(double meters) {
        this.wheelRadiusMeters = meters;
        return this;
    }

    /** Abort the drive test if chassis speed exceeds this, in m/s (default 4.5). */
    public SwerveIdConfig withMaxSpeedAbort(double metersPerSecond) {
        this.maxSpeedAbortMps = metersPerSecond;
        return this;
    }

    /** Abort the steer test if any steer motor exceeds this velocity (mechanism rot/s). */
    public SwerveIdConfig withMaxSteerVelocityAbort(double rotPerSec) {
        this.maxSteerVelocityAbort = rotPerSec;
        return this;
    }

    MotorModel driveMotorModel() {
        return driveMotorModel;
    }

    MotorModel steerMotorModel() {
        return steerMotorModel;
    }

    double travelMeters() {
        return travelMeters;
    }

    double wheelRadiusMeters() {
        return wheelRadiusMeters;
    }

    double maxSpeedAbortMps() {
        return maxSpeedAbortMps;
    }

    double maxSteerVelocityAbort() {
        return maxSteerVelocityAbort;
    }
}
