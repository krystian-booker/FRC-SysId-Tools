package com.frc2852.mechid;

/**
 * Physical parameters of the supported CTRE motors, used by the analysis tool to
 * model stator-current limiting (torque available at a given current cap) and to
 * sanity-check identified constants against the motor's theoretical limits.
 *
 * <p>Values match WPILib's {@code DCMotor} constants (trapezoidal commutation).
 */
public enum MotorModel {
    KRAKEN_X60("Kraken X60", 12.0, 7.09, 366.0, 2.0, 6000.0),
    KRAKEN_X44("Kraken X44", 12.0, 4.05, 275.0, 1.4, 7530.0),
    NEO_VORTEX("NEO Vortex", 12.0, 3.60, 211.0, 3.6, 6784.0);

    private final String displayName;
    private final double nominalVoltage;
    private final double stallTorqueNm;
    private final double stallCurrentAmps;
    private final double freeCurrentAmps;
    private final double freeSpeedRpm;

    MotorModel(String displayName, double nominalVoltage, double stallTorqueNm,
            double stallCurrentAmps, double freeCurrentAmps, double freeSpeedRpm) {
        this.displayName = displayName;
        this.nominalVoltage = nominalVoltage;
        this.stallTorqueNm = stallTorqueNm;
        this.stallCurrentAmps = stallCurrentAmps;
        this.freeCurrentAmps = freeCurrentAmps;
        this.freeSpeedRpm = freeSpeedRpm;
    }

    public String displayName() {
        return displayName;
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
