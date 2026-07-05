package com.frc2852.mechid;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Characterization for the CTRE swerve template ({@code CommandSwerveDrivetrain}).
 * Two routines:
 *
 * <ul>
 *   <li>{@link #driveRoutine()} — points the wheels forward and excites the drive
 *       motors (quasistatic ramp + steps), driving up to the configured travel distance
 *       and back. Run on open floor with the robot lined up along its runway.</li>
 *   <li>{@link #steerRoutine()} — excites the azimuth motors in place. Run it with the
 *       robot sitting on carpet: the dominant steering load is the wheel scrubbing
 *       against the floor under the robot's weight, so characterizing unloaded (on
 *       blocks) gives friction constants far below reality.</li>
 * </ul>
 *
 * All four modules are recorded; the analysis fits one gain set per motor role and
 * reports per-module spread.
 *
 * <pre>{@code
 * SwerveMechId swerveId = new SwerveMechId(drivetrain,
 *         new SwerveIdConfig(MotorModel.KRAKEN_X60, MotorModel.KRAKEN_X44)
 *                 .withTravelMeters(5.0)
 *                 .withWheelRadius(TunerConstants.kWheelRadius.in(Meters)));
 * }</pre>
 */
public class SwerveMechId {
    private final SwerveDrivetrain<TalonFX, TalonFX, ?> drivetrain;
    private final Subsystem requirement;
    private final SwerveIdConfig config;

    public <T extends SwerveDrivetrain<TalonFX, TalonFX, ?> & Subsystem> SwerveMechId(
            T drivetrain, SwerveIdConfig config) {
        this.drivetrain = drivetrain;
        this.requirement = drivetrain;
        this.config = config;
    }

    /** Guarded identification routine for the drive motors. Needs open floor space. */
    public Command driveRoutine() {
        int n = drivetrain.getModules().length;
        MechIdConfig mech = MechIdConfig.swerveDrive("swerve_drive",
                drivetrain.getModule(0).getDriveMotor(), config.driveMotorModel());
        TalonFX[] others = new TalonFX[n - 1];
        for (int i = 1; i < n; i++) {
            others[i - 1] = drivetrain.getModule(i).getDriveMotor();
        }
        mech.withFollowers(others)
                .withPositionRange(0.0, config.travelMeters())
                .withMaxVelocityAbort(config.maxSpeedAbortMps())
                .withPositionUnits("wheel_rot")
                .withSoftLimitManagement(false);

        List<SignalRecorder.Channel> channels = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            channels.add(new SignalRecorder.Channel("m" + i + "_drive",
                    new TalonFxMotor(drivetrain.getModule(i).getDriveMotor()), "module"));
        }
        return new MechIdRoutineCommand(mech, requirement, new TranslationIo(),
                channels, Map.of("wheelRadiusMeters", config.wheelRadiusMeters(),
                        "travelMeters", config.travelMeters(),
                        "moduleCount", (double) n));
    }

    /** Guarded identification routine for the steer motors. Run on carpet so the wheels see match loads. */
    public Command steerRoutine() {
        int n = drivetrain.getModules().length;
        MechIdConfig mech = MechIdConfig.swerveSteer("swerve_steer",
                drivetrain.getModule(0).getSteerMotor(), config.steerMotorModel());
        TalonFX[] others = new TalonFX[n - 1];
        for (int i = 1; i < n; i++) {
            others[i - 1] = drivetrain.getModule(i).getSteerMotor();
        }
        mech.withFollowers(others).withSoftLimitManagement(false);
        if (!Double.isNaN(config.maxSteerVelocityAbort())) {
            mech.withMaxVelocityAbort(config.maxSteerVelocityAbort());
        }

        List<SignalRecorder.Channel> channels = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            channels.add(new SignalRecorder.Channel("m" + i + "_steer",
                    new TalonFxMotor(drivetrain.getModule(i).getSteerMotor()), "module"));
        }
        return new MechIdRoutineCommand(mech, requirement, new SteerIo(),
                channels, Map.of("moduleCount", (double) n));
    }

    /**
     * Drives through {@code SysIdSwerveTranslation} (wheels held straight, voltage on
     * drive motors). Guard position is the signed distance traveled along the robot's
     * heading at routine start, so the travel range [0, travelMeters] matches the
     * physical runway the robot was lined up on.
     */
    private class TranslationIo implements RoutineIo {
        private final SwerveRequest.SysIdSwerveTranslation request = new SwerveRequest.SysIdSwerveTranslation();
        private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        private Translation2d start = Translation2d.kZero;
        private Translation2d heading = new Translation2d(1, 0);

        @Override
        public void onRoutineStart() {
            Pose2d pose = drivetrain.getState().Pose;
            start = pose.getTranslation();
            heading = new Translation2d(pose.getRotation().getCos(), pose.getRotation().getSin());
        }

        @Override
        public void applyVolts(double volts) {
            drivetrain.setControl(request.withVolts(Volts.of(volts)));
        }

        @Override
        public void stopNeutral() {
            drivetrain.setControl(idle);
        }

        @Override
        public void stopCoast() {
            drivetrain.setControl(idle);
        }

        @Override
        public double guardPosition() {
            Translation2d delta = drivetrain.getState().Pose.getTranslation().minus(start);
            return delta.getX() * heading.getX() + delta.getY() * heading.getY();
        }

        @Override
        public double guardVelocity() {
            var speeds = drivetrain.getState().Speeds;
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }

        @Override
        public double maxStatorCurrent() {
            double max = 0;
            for (var module : drivetrain.getModules()) {
                max = Math.max(max, Math.abs(module.getDriveMotor().getStatorCurrent().getValueAsDouble()));
            }
            return max;
        }
    }

    /** Drives through {@code SysIdSwerveSteerGains} (voltage on azimuth motors). */
    private class SteerIo implements RoutineIo {
        private final SwerveRequest.SysIdSwerveSteerGains request = new SwerveRequest.SysIdSwerveSteerGains();
        private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        private final List<StatusSignal<AngularVelocity>> steerVelocities = new ArrayList<>();

        SteerIo() {
            for (var module : drivetrain.getModules()) {
                steerVelocities.add(module.getSteerMotor().getVelocity());
            }
        }

        @Override
        public void applyVolts(double volts) {
            drivetrain.setControl(request.withVolts(Volts.of(volts)));
        }

        @Override
        public void stopNeutral() {
            drivetrain.setControl(idle);
        }

        @Override
        public void stopCoast() {
            drivetrain.setControl(idle);
        }

        @Override
        public double guardPosition() {
            return 0.0; // continuous rotation, no travel guard
        }

        @Override
        public double guardVelocity() {
            double max = 0;
            for (StatusSignal<AngularVelocity> v : steerVelocities) {
                max = Math.max(max, Math.abs(v.getValueAsDouble()));
            }
            return max;
        }

        @Override
        public double maxStatorCurrent() {
            double max = 0;
            for (var module : drivetrain.getModules()) {
                max = Math.max(max, Math.abs(module.getSteerMotor().getStatorCurrent().getValueAsDouble()));
            }
            return max;
        }
    }
}
