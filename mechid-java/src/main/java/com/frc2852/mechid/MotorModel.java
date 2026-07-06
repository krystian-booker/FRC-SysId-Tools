package com.frc2852.mechid;

/**
 * Physical parameters of the supported motors, used by the analysis tool to model
 * stator-current limiting (torque available at a given current cap) and to
 * sanity-check identified constants against the motor's theoretical limits.
 *
 * <p>Values match WPILib's {@code DCMotor} constants. The {@code _FOC} variants are
 * the equivalent linear model of the same motor running field-oriented control (more
 * torque per amp, slightly lower free speed) — pick the variant matching how the
 * mechanism actually runs, and pair it with {@link MechIdConfig#withFoc}.
 */
public enum MotorModel {
    KRAKEN_X60("Kraken X60", 12.0, 7.09, 366.0, 2.0, 6000.0, false),
    KRAKEN_X60_FOC("Kraken X60 (FOC)", 12.0, 9.37, 483.0, 2.0, 5800.0, true),
    KRAKEN_X44("Kraken X44", 12.0, 4.11, 279.0, 2.0, 7758.0, false),
    KRAKEN_X44_FOC("Kraken X44 (FOC)", 12.0, 5.01, 329.0, 2.0, 7368.0, true),
    NEO_VORTEX("NEO Vortex", 12.0, 3.60, 211.0, 3.6, 6784.0, false);

    private final String displayName;
    private final double nominalVoltage;
    private final double stallTorqueNm;
    private final double stallCurrentAmps;
    private final double freeCurrentAmps;
    private final double freeSpeedRpm;
    private final boolean foc;

    MotorModel(String displayName, double nominalVoltage, double stallTorqueNm,
            double stallCurrentAmps, double freeCurrentAmps, double freeSpeedRpm, boolean foc) {
        this.displayName = displayName;
        this.nominalVoltage = nominalVoltage;
        this.stallTorqueNm = stallTorqueNm;
        this.stallCurrentAmps = stallCurrentAmps;
        this.freeCurrentAmps = freeCurrentAmps;
        this.freeSpeedRpm = freeSpeedRpm;
        this.foc = foc;
    }

    public String displayName() {
        return displayName;
    }

    /** True for the field-oriented-control variants of the CTRE motors. */
    public boolean isFoc() {
        return foc;
    }

    public double nominalVoltage() {
        return nominalVoltage;
    }

    public double stallTorqueNm() {
        return stallTorqueNm;
    }

    public double stallCurrentAmps() {
        return stallCurrentAmps;
    }

    public double freeCurrentAmps() {
        return freeCurrentAmps;
    }

    public double freeSpeedRpm() {
        return freeSpeedRpm;
    }

    /** Winding resistance in ohms, derived from stall conditions. */
    public double resistanceOhms() {
        return nominalVoltage / stallCurrentAmps;
    }

    /** Torque constant in N·m per amp of stator current. */
    public double torqueConstantNmPerAmp() {
        return stallTorqueNm / stallCurrentAmps;
    }

    /** Back-EMF constant in volts per rotor rotation-per-second. */
    public double emfConstantVoltsPerRps() {
        double freeSpeedRps = freeSpeedRpm / 60.0;
        return (nominalVoltage - freeCurrentAmps * resistanceOhms()) / freeSpeedRps;
    }
}
