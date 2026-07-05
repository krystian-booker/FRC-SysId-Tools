package com.frc2852.mechid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Convenience base class: extend this instead of {@link SubsystemBase} and describe
 * your mechanism once, and the subsystem gains a ready-to-bind {@link #mechIdRoutine()}
 * command.
 *
 * <pre>{@code
 * public class Turret extends CharacterizableSubsystem {
 *     private final TalonFX motor = ...;
 *
 *     @Override
 *     protected MechIdConfig configureMechId() {
 *         return MechIdConfig.turret("turret", motor, MotorModel.KRAKEN_X60)
 *                 .withPositionRange(-0.5, 0.5)
 *                 .withRotorToMechanismRatio(50.0)
 *                 .withMaxVelocityAbort(3.0);
 *     }
 * }
 *
 * // RobotContainer:
 * RobotModeTriggers.test().and(controller.povUp()).toggleOnTrue(turret.mechIdRoutine());
 * }</pre>
 *
 * <p>{@code configureMechId()} is called lazily on first use, so motors constructed in
 * your subsystem's constructor are safe to reference.
 */
public abstract class CharacterizableSubsystem extends SubsystemBase {
    private MechId mechId;

    /** Describe this subsystem's mechanism. Called once, on first use. */
    protected abstract MechIdConfig configureMechId();

    /** The {@link MechId} for this subsystem, creating it on first call. */
    public final synchronized MechId getMechId() {
        if (mechId == null) {
            mechId = new MechId(configureMechId(), this);
        }
        return mechId;
    }

    /** The full guarded identification routine for this subsystem. Bind to a test-mode button. */
    public final Command mechIdRoutine() {
        return getMechId().routine();
    }
}
