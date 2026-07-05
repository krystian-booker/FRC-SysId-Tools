package com.frc2852.mechid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * {@link MechMotor} backed by a REV SparkFlex (NEO Vortex).
 *
 * <p>Differences from TalonFX handled here:
 * <ul>
 *   <li>No onboard acceleration signal — {@link #acceleration()} returns NaN and the
 *       analysis differentiates velocity instead.</li>
 *   <li>No coast-override control: {@link #stopCoast()} respects the configured idle
 *       mode, so the coast-down test is skipped for REV-led mechanisms.</li>
 *   <li>Status frame periods are raised for the recording and restored to REV
 *       defaults afterward.</li>
 * </ul>
 */
class SparkFlexMotor implements MechMotor {
    private static final BaseStatusSignal[] NO_SIGNALS = new BaseStatusSignal[0];

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private boolean softLimitsSaved = false;
    private boolean savedFwdEnabled;
    private boolean savedRevEnabled;
    private double savedFwdLimit;
    private double savedRevLimit;

    SparkFlexMotor(SparkFlex motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
    }

    SparkFlex device() {
        return motor;
    }

    @Override
    public String vendorName() {
        return "SparkFlex";
    }

    @Override
    public void applyVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stopNeutral() {
        motor.stopMotor();
    }

    @Override
    public void stopCoast() {
        motor.stopMotor();
    }

    @Override
    public boolean supportsCoast() {
        return false;
    }

    @Override
    public void setFoc(boolean foc) {
        // Not applicable to SparkFlex.
    }

    @Override
    public double position() {
        return encoder.getPosition();
    }

    @Override
    public double velocity() {
        return encoder.getVelocity();
    }

    @Override
    public double acceleration() {
        return Double.NaN;
    }

    @Override
    public double appliedVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double statorCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double supplyCurrent() {
        // REV reports output (stator) current; supply ~= output current * duty cycle.
        return Math.abs(motor.getOutputCurrent() * motor.getAppliedOutput());
    }

    @Override
    public BaseStatusSignal[] bulkRefreshSignals() {
        return NO_SIGNALS;
    }

    @Override
    public void beginHighRateSampling(double hz) {
        int periodMs = (int) Math.max(5, Math.round(1000.0 / hz));
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.signals
                .appliedOutputPeriodMs(periodMs)
                .busVoltagePeriodMs(periodMs)
                .outputCurrentPeriodMs(periodMs)
                .primaryEncoderPositionPeriodMs(periodMs)
                .primaryEncoderVelocityPeriodMs(periodMs);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void endHighRateSampling() {
        // Back to REV defaults (applied output 10 ms, everything else 20 ms).
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.signals
                .appliedOutputPeriodMs(10)
                .busVoltagePeriodMs(10)
                .outputCurrentPeriodMs(20)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityPeriodMs(20);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void saveAndDisableSoftLimits() {
        var accessor = motor.configAccessor.softLimit;
        savedFwdEnabled = accessor.getForwardSoftLimitEnabled();
        savedRevEnabled = accessor.getReverseSoftLimitEnabled();
        savedFwdLimit = accessor.getForwardSoftLimit();
        savedRevLimit = accessor.getReverseSoftLimit();
        softLimitsSaved = true;

        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void restoreSoftLimits() {
        if (!softLimitsSaved) {
            return;
        }
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.softLimit
                .forwardSoftLimitEnabled(savedFwdEnabled)
                .reverseSoftLimitEnabled(savedRevEnabled)
                .forwardSoftLimit(savedFwdLimit)
                .reverseSoftLimit(savedRevLimit);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        softLimitsSaved = false;
    }

    @Override
    public void saveAppliedControl() {
        // REV has no readable control request; synchronized REV motors are simply
        // stopped after the routine and the subsystem re-commands them.
    }

    @Override
    public void restoreAppliedControl() {
        motor.stopMotor();
    }

    @Override
    public double rotorToMechanismRatio() {
        double factor = motor.configAccessor.encoder.getPositionConversionFactor();
        return factor > 0 ? 1.0 / factor : Double.NaN;
    }

    @Override
    public boolean ratioTrusted() {
        return motor.configAccessor.encoder.getPositionConversionFactor() > 0;
    }

    @Override
    public void writeDeviceMeta(JsonBuilder json) {
        var accessor = motor.configAccessor;
        json.beginObject()
                .field("vendor", vendorName())
                .field("deviceId", (long) motor.getDeviceId())
                .field("firmware", motor.getFirmwareString())
                .field("inverted", accessor.getInverted())
                .field("idleMode", accessor.getIdleMode().toString())
                .field("positionConversionFactor", accessor.encoder.getPositionConversionFactor())
                .field("velocityConversionFactor", accessor.encoder.getVelocityConversionFactor());

        json.key("currentLimits").beginObject()
                .field("smartCurrentLimit", (long) accessor.getSmartCurrentLimit())
                .endObject();

        json.key("closedLoop").beginObject()
                .field("kP", accessor.closedLoop.getP())
                .field("kI", accessor.closedLoop.getI())
                .field("kD", accessor.closedLoop.getD())
                .endObject();

        json.endObject();
    }
}
