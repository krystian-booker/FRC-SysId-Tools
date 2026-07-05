package com.frc2852.mechid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

/** {@link MechMotor} backed by a CTRE TalonFX (Kraken X60/X44). */
class TalonFxMotor implements MechMotor {
    private final TalonFX motor;
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final NeutralOut neutralOut = new NeutralOut();
    private final CoastOut coastOut = new CoastOut();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final BaseStatusSignal[] signals;
    private final double[] restoreFrequencies;

    private final SoftwareLimitSwitchConfigs savedSoftLimits = new SoftwareLimitSwitchConfigs();
    private boolean softLimitsSaved = false;
    private ControlRequest savedControl = null;

    TalonFxMotor(TalonFX motor) {
        this.motor = motor;
        position = motor.getPosition();
        velocity = motor.getVelocity();
        acceleration = motor.getAcceleration();
        motorVoltage = motor.getMotorVoltage();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        signals = new BaseStatusSignal[] {
                position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent
        };
        restoreFrequencies = new double[signals.length];
    }

    TalonFX device() {
        return motor;
    }

    @Override
    public String vendorName() {
        return "TalonFX";
    }

    @Override
    public void applyVolts(double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stopNeutral() {
        motor.setControl(neutralOut);
    }

    @Override
    public void stopCoast() {
        motor.setControl(coastOut);
    }

    @Override
    public boolean supportsCoast() {
        return true;
    }

    @Override
    public void setFoc(boolean foc) {
        voltageOut.withEnableFOC(foc);
    }

    @Override
    public double position() {
        return position.getValueAsDouble();
    }

    @Override
    public double velocity() {
        return velocity.getValueAsDouble();
    }

    @Override
    public double acceleration() {
        return acceleration.getValueAsDouble();
    }

    @Override
    public double appliedVolts() {
        return motorVoltage.getValueAsDouble();
    }

    @Override
    public double statorCurrent() {
        return statorCurrent.getValueAsDouble();
    }

    @Override
    public double supplyCurrent() {
        return supplyCurrent.getValueAsDouble();
    }

    @Override
    public BaseStatusSignal[] bulkRefreshSignals() {
        return signals;
    }

    @Override
    public void beginHighRateSampling(double hz) {
        for (int i = 0; i < signals.length; i++) {
            restoreFrequencies[i] = signals[i].getAppliedUpdateFrequency();
        }
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

    @Override
    public void endHighRateSampling() {
        for (int i = 0; i < signals.length; i++) {
            if (restoreFrequencies[i] > 0) {
                signals[i].setUpdateFrequency(restoreFrequencies[i]);
            }
        }
    }

    @Override
    public void saveAndDisableSoftLimits() {
        var status = motor.getConfigurator().refresh(savedSoftLimits);
        if (status.isOK()) {
            softLimitsSaved = true;
            motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withReverseSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(savedSoftLimits.ForwardSoftLimitThreshold)
                    .withReverseSoftLimitThreshold(savedSoftLimits.ReverseSoftLimitThreshold));
        } else {
            DriverStation.reportWarning(
                    "[MechID] Could not read soft limit config; leaving soft limits as-is", false);
        }
    }

    @Override
    public void restoreSoftLimits() {
        if (softLimitsSaved) {
            motor.getConfigurator().apply(savedSoftLimits);
            softLimitsSaved = false;
        }
    }

    @Override
    public void saveAppliedControl() {
        savedControl = motor.getAppliedControl();
    }

    @Override
    public void restoreAppliedControl() {
        if (savedControl != null) {
            motor.setControl(savedControl);
            savedControl = null;
        } else {
            motor.setControl(neutralOut);
        }
    }

    @Override
    public double rotorToMechanismRatio() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        motor.getConfigurator().refresh(cfg);
        return cfg.Feedback.RotorToSensorRatio * cfg.Feedback.SensorToMechanismRatio;
    }

    @Override
    public boolean ratioTrusted() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        motor.getConfigurator().refresh(cfg);
        return !cfg.Feedback.FeedbackSensorSource.toString().contains("Remote");
    }

    @Override
    public void writeDeviceMeta(JsonBuilder json) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        motor.getConfigurator().refresh(cfg);
        json.beginObject()
                .field("vendor", vendorName())
                .field("deviceId", (long) motor.getDeviceID())
                .field("canBus", motor.getNetwork().getName())
                .field("firmwareVersion", (long) motor.getVersion().getValue())
                .field("rotorToSensorRatio", cfg.Feedback.RotorToSensorRatio)
                .field("sensorToMechanismRatio", cfg.Feedback.SensorToMechanismRatio)
                .field("feedbackSource", cfg.Feedback.FeedbackSensorSource.toString())
                .field("neutralMode", cfg.MotorOutput.NeutralMode.toString())
                .field("inverted", cfg.MotorOutput.Inverted.toString());

        json.key("currentLimits").beginObject()
                .field("statorLimitEnabled", cfg.CurrentLimits.StatorCurrentLimitEnable)
                .field("statorLimit", cfg.CurrentLimits.StatorCurrentLimit)
                .field("supplyLimitEnabled", cfg.CurrentLimits.SupplyCurrentLimitEnable)
                .field("supplyLimit", cfg.CurrentLimits.SupplyCurrentLimit)
                .endObject();

        json.key("slot0").beginObject()
                .field("kS", cfg.Slot0.kS)
                .field("kV", cfg.Slot0.kV)
                .field("kA", cfg.Slot0.kA)
                .field("kG", cfg.Slot0.kG)
                .field("kP", cfg.Slot0.kP)
                .field("kI", cfg.Slot0.kI)
                .field("kD", cfg.Slot0.kD)
                .field("gravityType", cfg.Slot0.GravityType.toString())
                .endObject();

        json.key("motionMagic").beginObject()
                .field("cruiseVelocity", cfg.MotionMagic.MotionMagicCruiseVelocity)
                .field("acceleration", cfg.MotionMagic.MotionMagicAcceleration)
                .endObject();

        json.endObject();
    }
}
