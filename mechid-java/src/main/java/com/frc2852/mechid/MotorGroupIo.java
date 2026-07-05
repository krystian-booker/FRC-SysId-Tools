package com.frc2852.mechid;

import java.util.List;

/**
 * Drives a mechanism's motor group with open-loop voltage: the leader and every
 * {@code withSynchronizedMotors} motor receive the identical command each cycle
 * (follower-configured motors mirror on their own and are never commanded). Guards
 * read the leader; the current abort watches every motor on the mechanism.
 *
 * <p>On routine start the synchronized motors' active control requests are saved, and
 * software limit switches are disabled on all commanded motors; both are restored when
 * the routine ends.
 */
class MotorGroupIo implements RoutineIo {
    private final MechIdConfig config;
    private final MechMotor leader;
    private final List<MechMotor> commanded;
    private final List<MechMotor> all;

    MotorGroupIo(MechIdConfig config) {
        this.config = config;
        this.leader = config.leader();
        this.commanded = config.commandedMotors();
        this.all = config.allMotors();
    }

    @Override
    public void applyVolts(double volts) {
        for (MechMotor m : commanded) {
            m.applyVolts(volts);
        }
    }

    @Override
    public void stopNeutral() {
        for (MechMotor m : commanded) {
            m.stopNeutral();
        }
    }

    @Override
    public void stopCoast() {
        for (MechMotor m : commanded) {
            m.stopCoast();
        }
    }

    @Override
    public double guardPosition() {
        return leader.position();
    }

    @Override
    public double guardVelocity() {
        return leader.velocity();
    }

    @Override
    public double maxStatorCurrent() {
        double max = 0;
        for (MechMotor m : all) {
            max = Math.max(max, Math.abs(m.statorCurrent()));
        }
        return max;
    }

    @Override
    public void onRoutineStart() {
        for (MechMotor m : commanded) {
            m.setFoc(config.useFoc());
        }
        for (MechMotor m : config.synchronizedMotors()) {
            m.saveAppliedControl();
        }
        if (config.manageSoftLimits() && config.type().isTravelLimited()) {
            for (MechMotor m : commanded) {
                m.saveAndDisableSoftLimits();
            }
        }
    }

    @Override
    public void onRoutineEnd() {
        for (MechMotor m : commanded) {
            m.restoreSoftLimits();
        }
        for (MechMotor m : config.synchronizedMotors()) {
            m.restoreAppliedControl();
        }
    }
}
