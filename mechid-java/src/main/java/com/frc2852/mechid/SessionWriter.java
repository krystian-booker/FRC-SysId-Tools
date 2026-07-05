package com.frc2852.mechid;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Owns one recording session's output folder: creates
 * {@code <root>/<name>_<timestamp>/}, writes {@code meta.json} describing the
 * mechanism and the as-configured device state, and hands out the data.csv path.
 *
 * <p>Root defaults to {@code Filesystem.getOperatingDirectory()/mechid}, which is
 * {@code /home/lvuser/mechid} on the roboRIO and the project directory in simulation.
 */
class SessionWriter {
    static final String LIBRARY_VERSION = "2026.1.0";
    static final int FORMAT_VERSION = 1;

    private final MechIdConfig config;
    private final Path sessionDir;

    SessionWriter(MechIdConfig config) throws IOException {
        this.config = config;
        String root = config.outputRootOverride() != null
                ? config.outputRootOverride()
                : Filesystem.getOperatingDirectory().getAbsolutePath() + "/mechid";
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        this.sessionDir = Path.of(root, config.name() + "_" + timestamp);
        Files.createDirectories(sessionDir);
    }

    Path sessionDir() {
        return sessionDir;
    }

    Path dataCsv() {
        return sessionDir.resolve("data.csv");
    }

    /**
     * Writes meta.json. Reads back each motor's applied configuration so the analysis
     * tool knows gear ratios, existing gains, and existing current limits without the
     * user re-typing them.
     *
     * @param motorColumnNames CSV column prefixes in recording order — must match the
     *                         motors given, which follow {@code config.allMotors()}
     *                         order for single-mechanism routines
     * @param motors           the recorded motors in the same order
     * @param roles            role per motor ("leader", "synchronized", "follower", ...)
     */
    void writeMeta(List<String> motorColumnNames, List<MechMotor> motors, List<String> roles,
            Map<String, Double> extra) throws IOException {
        JsonBuilder json = new JsonBuilder();
        json.beginObject()
                .field("formatVersion", (long) FORMAT_VERSION)
                .field("library", "MechID " + LIBRARY_VERSION)
                .field("name", config.name())
                .field("mechanismType", config.type().name())
                .field("pidDomain", config.type().pidDomain().name())
                .field("gravityModel", config.type().gravityModel().name())
                .field("positionUnits", config.positionUnits())
                .field("startedAt", LocalDateTime.now().toString());

        json.key("extra").beginObject();
        if (extra != null) {
            for (var entry : extra.entrySet()) {
                json.field(entry.getKey(), entry.getValue());
            }
        }
        json.endObject();

        json.key("motorModel").beginObject()
                .field("name", config.motorModel().name())
                .field("nominalVoltage", config.motorModel().nominalVoltage())
                .field("stallTorqueNm", config.motorModel().stallTorqueNm())
                .field("stallCurrentAmps", config.motorModel().stallCurrentAmps())
                .field("freeCurrentAmps", config.motorModel().freeCurrentAmps())
                .field("freeSpeedRpm", config.motorModel().freeSpeedRpm())
                .endObject();

        json.key("routine").beginObject()
                .field("quasistaticRampVoltsPerSec", config.quasistaticRampRate())
                .field("quasistaticMaxVoltage", config.quasistaticMaxVoltage())
                .arrayField("dynamicStepVoltages", config.dynamicStepVoltages())
                .field("dynamicStepHoldSec", config.dynamicStepHoldSec())
                .field("coastdownEnabled", config.coastdownEnabled())
                .field("sampleRateHz", config.sampleRateHz())
                .field("useFoc", config.useFoc())
                .endObject();

        json.key("envelope").beginObject()
                .field("minPosition", config.minPosition())
                .field("maxPosition", config.maxPosition())
                .field("maxVelocityAbort", config.maxVelocityAbort())
                .field("statorCurrentAbort", config.statorCurrentAbort())
                .field("armHorizontalPosition", config.armHorizontalPosition())
                .endObject();

        // rotorToMechanismRatio: explicit override wins; otherwise derived from the
        // leader's device configuration (flagged untrusted when the device can't know,
        // e.g. TalonFX with RemoteCANcoder feedback).
        boolean userRatio = !Double.isNaN(config.rotorToMechanismRatio());
        double ratio = userRatio ? config.rotorToMechanismRatio() : config.leader().rotorToMechanismRatio();
        json.key("gearing").beginObject()
                .field("rotorToMechanismRatio", ratio)
                .field("source", userRatio ? "user" : "deviceConfig")
                .field("ratioTrusted", userRatio || config.leader().ratioTrusted())
                .endObject();

        json.arrayField("motors", motorColumnNames);
        json.arrayField("motorRoles", roles);

        json.key("devices").beginObject();
        for (int i = 0; i < motors.size() && i < motorColumnNames.size(); i++) {
            json.key(motorColumnNames.get(i));
            motors.get(i).writeDeviceMeta(json);
        }
        json.endObject();

        json.endObject();
        Files.writeString(sessionDir.resolve("meta.json"), json.toString());
    }
}
