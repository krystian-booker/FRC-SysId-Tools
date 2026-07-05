# MechID

Reusable system identification for FRC mechanisms on CTRE TalonFX (Kraken X60/X44) and
REV SparkFlex (NEO Vortex). Describe a mechanism in a few lines of robot code, run one
guarded on-robot routine, and get back **feedforward (kS/kV/kA/kG), PID gains, Motion
Magic limits, and a recommended stator current limit** — with an interactive HTML report
showing exactly what each current limit costs you in performance.

```
robot (Java)                 laptop (Python + MATLAB)
────────────────             ─────────────────────────────
MechIdConfig  ──► routine ──► /home/lvuser/mechid/<session>/
                                    │  mechid pull --team 2852
                                    ▼
                              mechid analyze <session>
                                    │
                                    ▼
                              constants + report.html
```

Supported mechanism types: **elevator** (position, constant gravity), **turret**
(position, no gravity), **pivoting arm** (position, cosine gravity), **flywheel /
velocity rollers**, and the **CTRE swerve template** (drive + steer).

## Robot side (mechid-java)

### Install

Until this is published to GitHub Pages (vendordep template in `vendordep/`), use the
local maven flow:

```bash
cd vendor-lib && ./gradlew publishToMavenLocal
```

then in the robot project's `build.gradle`:

```groovy
repositories { mavenLocal() }
dependencies { implementation 'com.frc2852.mechid:mechid-java:2026.1.0' }
```

### Describe a mechanism

Option A — extend the base class (new subsystems):

```java
public class Turret extends CharacterizableSubsystem {
    @Override
    protected MechIdConfig configureMechId() {
        return MechIdConfig.turret("turret", motor, MotorModel.KRAKEN_X60)
                .withPositionRange(-0.5, 0.5)          // mechanism rotations
                .withRotorToMechanismRatio(50.0)        // REQUIRED with RemoteCANcoder feedback
                .withMaxVelocityAbort(2.0);
    }
}
// turret.mechIdRoutine() is now available
```

Option B — composition (any existing subsystem, no base-class change):

```java
private final MechId mechId = new MechId(
        MechIdConfig.flywheel("flywheel", leaderMotor, MotorModel.KRAKEN_X60)
                .withFollowers(followerMotor)           // recorded, never commanded
                .withMaxVelocityAbort(100.0),
        this);

public Command mechIdRoutine() { return mechId.routine(); }
```

Factories with tuned excitation defaults: `MechIdConfig.elevator(...)`, `.turret(...)`,
`.arm(...)`, `.flywheel(...)` — each with overloads for `TalonFX` and `SparkFlex`
leaders. Everything is overridable (`withQuasistaticRamp`, `withDynamicSteps`,
`withSampleRate`, `withStatorCurrentAbort`, ...).

**Multi-motor mechanisms** — describe the other motors by how they're controlled:

- `withFollowers(...)` — motors already configured with vendor follower control
  (Phoenix `Follower` request, REV `config.follow()`). They mirror the leader's output
  on their own during the routine, so MechID records them but never commands them.
- `withSynchronizedMotors(...)` — independent motors on the same mechanism that are
  *not* followers. MechID drives them with the identical voltage as the leader (each
  motor's own inversion config must make +V = mechanism-forward) and restores their
  previous control request when the routine ends.

Every motor is recorded either way, so the current-limit analysis budgets the whole
group. Supports any number of motors.

Swerve (CTRE template — `CommandSwerveDrivetrain`):

```java
private final SwerveMechId mechId = new SwerveMechId(this,
        new SwerveIdConfig(MotorModel.KRAKEN_X60, MotorModel.KRAKEN_X44)  // drive, steer
                .withTravelMeters(4.0)
                .withWheelRadius(TunerConstants.FrontLeft.WheelRadius));
// mechId.driveRoutine()  — robot on open floor, ~4 m runway ahead
// mechId.steerRoutine()  — robot sitting on carpet (wheels must carry match loads)
```

All four modules are recorded and fit individually; the analysis reports one shared gain
set per role (that is also what CTRE's API applies) plus the per-module spread, so one
dragging module stands out immediately.

### Bind and run

```java
RobotModeTriggers.test().and(controller.povUp()).toggleOnTrue(turret.mechIdRoutine());
```

Enable **test mode**, press the button, stand clear. The routine runs a quasistatic
voltage ramp (forward and reverse), dynamic voltage steps at several amplitudes, and a
coast-down for velocity mechanisms — 30–120 s total. Press again to cancel; partial data
is still saved.

Safety, handled for you:

- Position-limited mechanisms reverse with an 8% margin before the configured range and
  hard-abort if they ever escape it.
- Aborts on over-velocity, sustained over-current, and routine timeout.
- TalonFX software limit switches are disabled during the routine and restored after
  (MechID enforces its own limits), so soft limits don't fight the test.
- Everything stops on disable, like any WPILib command.

Data lands in `/home/lvuser/mechid/<name>_<timestamp>/` — `data.csv` (200 Hz signals for
every motor) plus `meta.json` (mechanism description **and** the as-configured device
state: gear ratios, existing gains, existing current limits — the analysis never asks
you to retype them).

## Laptop side (analysis/)

```bash
cd analysis && uv sync
uv run mechid pull --team 2852        # copies sessions from the RIO
uv run mechid analyze mechid_data/turret_20260704_193012
```

Two interchangeable engines, cross-validated against each other by the test suite:

- **MATLAB** (default when installed): `tfest` (System Identification Toolbox) for the
  dynamics, `lqr` (Control System Toolbox) for gains.
- **Python** (numpy/scipy): same model, same schema — for laptops without MATLAB.

Run `--engine both` to see them side by side. See [analysis/README.md](analysis/README.md)
for all flags.

You get, per session:

- console summary + `results_*.json`
- `report.html` — self-contained, open anywhere:
  - quasistatic fit scatter (data quality at a glance, R²)
  - measured-vs-model validation overlays on the step tests
  - a **paste-ready Phoenix 6 `Slot0Configs`/MotionMagic/CurrentLimits snippet**
  - the **current-limit explorer**: a slider over stator limits showing the simulated
    motion, per-motor stator current, and total supply draw at each limit, next to the
    performance-vs-limit curve with the recommended knee marked (default: the smallest
    limit that keeps ≥95% of unlimited performance)

### How the current-limit recommendation works

From the identified model plus the motor's electrical constants (R, torque and EMF
constants from the Kraken datasheets), the tool simulates a representative motion — a
full-travel move for position mechanisms, spin-up for flywheels, a 3 m sprint for swerve
drive — under each candidate stator limit. Current limiting caps torque, so low limits
stretch the move time; the curve flattens as the limit rises. The recommendation is the
knee of that curve, and the report shows peak *supply* draw per limit so you can budget
the whole robot against the battery.

## Conventions & limitations

- All positions/velocities are **mechanism units** — whatever the controller reports
  after its conversion (`SensorToMechanismRatio` / REV conversion factors). Identified
  gains are therefore in native units and paste straight into Phoenix 6 Slot0.
- Gains target **Voltage control requests** (`PositionVoltage`, `VelocityVoltage`,
  `MotionMagicVoltage`, ...). The report includes approximate `TorqueCurrentFOC`
  (amp-domain) conversions. REV's onboard closed loop outputs duty cycle: divide the
  voltage-domain kP/kD by 12 and supply kS/kV/kG as arbitrary feedforward in volts.
- SparkFlex specifics: REV provides no acceleration signal (the analysis differentiates
  velocity instead — slightly noisier kA), cannot force coast mode (the coast-down test
  is skipped), and its status frame periods are raised during the recording and
  restored to REV defaults afterward. Set the encoder conversion factors so position
  and velocity are in consistent mechanism units (rotations and rot/s).
- Robot projects need the vendordep for each motor vendor they characterize
  (Phoenix 6 always; REVLib when using the SparkFlex overloads).
- Arms follow Phoenix's `Arm_Cosine` convention (position 0 = horizontal); use
  `withArmHorizontalPosition` if your zero differs.
- With `RemoteCANcoder` feedback the device config cannot reveal the motor gearing —
  set `withRotorToMechanismRatio(...)` or the current-limit model falls back to an
  estimate (flagged in the report).
- Followers are assumed to be configured with `Follower` control by your subsystem;
  MechID records them (current budget) but never commands them.
- kI is intentionally 0: with good feedforward, integral gain mostly adds windup. Add a
  small kI on the robot only if you see steady-state error.
- After a routine, the raised signal update rates on the recorded motors are restored
  to their previous values; signals that were disabled via `optimizeBusUtilization`
  remain at the recording rate until the next power cycle (harmless during a tuning
  session).

## Repository layout

```
mechid-java/   Java library (compileOnly against WPILib 2026 + Phoenix 6 26.x)
analysis/      Python CLI + MATLAB scripts (uv project: `uv sync`)
vendordep/     MechID.json template for GitHub-Pages hosting (QuestNav-style)
```

## Releasing for other teams (later)

1. Push this repo to GitHub (e.g. `frc2852/MechID`).
2. Publish the maven artifact to GitHub Pages (`./gradlew publish` into a `maven/`
   folder served by Pages) and host `vendordep/MechID.json` beside it.
3. Teams install via WPILib VS Code → *Manage Vendor Libraries* → *Install new library
   (online)* with the JSON URL, and `uv tool install` the analysis CLI.

## Year-over-year checklist

1. Bump `wpilibVersion`/`phoenix6Version` in `mechid-java/build.gradle`, rebuild.
2. Bump `frcYear` + versions in `vendordep/MechID.json`.
3. Re-run `analysis` tests (`uv run pytest`) — they validate the whole pipeline against
   synthetic ground truth, including the MATLAB engine when installed.
