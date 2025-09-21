package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Method;
import java.util.List;
import java.util.StringJoiner;

@Config
@TeleOp(name = "AprilTag Calibrator (Compat)", group = "Tools")
public class AprilTagCalibrator extends LinearOpMode {

    // === Dashboard-tunable values ===
    public static String WEBCAM_NAME = "Webcam 1"; // must match RC config

    // Your target IDs
    public static int TARGET_ID_1 = 21; // gpp
    public static int TARGET_ID_2 = 22; // pgp
    public static int TARGET_ID_3 = 23; // ppg
    public static int TARGET_ID_4 = 20; // blue
    public static int TARGET_ID_5 = 24; // red

    // Physical tag black-square size (meters). FTC big tag ~0.166 m.
    public static double TAG_SIZE_M = 0.166;

    // Intrinsics you will tune:
    public static double FX = 800.0;
    public static double FY = 800.0;
    public static double CX = 320.0;   // set near width/2
    public static double CY = 240.0;   // set near height/2

    // For quick accuracy checks at the field: enter a measured distance (m)
    public static double KNOWN_DISTANCE_M = 0.50;

    // Decimation tradeoff (speed vs. range). 1=far/slow, 3=near/fast.
    public static float DECIMATION = 2.0f;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // toggles
    private boolean liveViewOn = true;
    private boolean aPrev = false, yPrev = false;

    @Override
    public void runOpMode() {
        initVision();  // builds with current FX/FY/CX/CY if supported

        FtcDashboard dash = FtcDashboard.getInstance();

        telemetry.addLine("Open FTC Dashboard → Camera Stream.");
        telemetry.addLine("Use Config sliders to tune fx/fy/cx/cy.");
        telemetry.addLine("Press  Y  to rebuild with current values.");
        telemetry.addLine("Press  A  to toggle Driver Hub preview.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleButtons();

            // Get latest detections
            List<AprilTagDetection> dets = aprilTag.getDetections();
            AprilTagDetection target = pickTarget(dets);

            // (Helper) show all IDs seen this frame
            if (!dets.isEmpty()) {
                StringJoiner sj = new StringJoiner(", ");
                for (AprilTagDetection d : dets) sj.add(String.valueOf(d.id));
                telemetry.addData("IDs seen", sj.toString());
            } else {
                telemetry.addLine("IDs seen: none");
            }

            telemetry.addLine(liveViewOn ? "LiveView: ON (A toggles)" : "LiveView: OFF (A toggles)");
            telemetry.addData("Intrinsics", "fx=%.1f fy=%.1f cx=%.1f cy=%.1f", FX, FY, CX, CY);

            TelemetryPacket packet = new TelemetryPacket();
            if (target != null) {
                addPoseTelemetry(target, packet);
            } else {
                telemetry.addLine("No *target* tag in view...");
                packet.put("status", "no target tag");
            }
            telemetry.update();
            dash.sendTelemetryPacket(packet);

            sleep(15);
        }

        // Clean up
        try { visionPortal.close(); } catch (Throwable ignore) {}
    }

    // -------------------- Vision setup / rebuild --------------------

    private void initVision() {
        // Build AprilTag processor (SDK-compatible).
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);

        // Optional features via reflection (compile-safe on old SDKs)
        callIfExists(builder, "setTagSize", new Class[]{double.class}, TAG_SIZE_M);
        callIfExists(builder, "setLensIntrinsics",
                new Class[]{double.class, double.class, double.class, double.class},
                FX, FY, CX, CY);

        aprilTag = builder.build();

        // Decimation via reflection
        callIfExists(aprilTag, "setDecimation", new Class[]{float.class}, DECIMATION);

        // Build VisionPortal
        VisionPortal.Builder vp = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag);

        // Live view is present on most SDKs; enable if available
        callIfExists(vp, "enableLiveView", new Class[]{boolean.class}, true);
        callIfExists(vp, "setAutoStopLiveView", new Class[]{boolean.class}, false);

        visionPortal = vp.build();
    }

    /** Rebuild the AprilTag processor (and portal) to apply current FX/FY/CX/CY without restarting OpMode. */
    private void rebuildWithCurrentIntrinsics() {
        try { visionPortal.close(); } catch (Throwable ignore) {}
        initVision();
    }

    // -------------------- Telemetry helpers --------------------

    private AprilTagDetection pickTarget(List<AprilTagDetection> dets) {
        AprilTagDetection pick = null;
        for (AprilTagDetection d : dets) {
            if (d.id == TARGET_ID_1
                    || d.id == TARGET_ID_2
                    || d.id == TARGET_ID_3
                    || d.id == TARGET_ID_4
                    || d.id == TARGET_ID_5) {
                pick = d; // keep last seen that matches any of your 5 IDs
            }
        }
        return pick;
    }

    private void addPoseTelemetry(AprilTagDetection d, TelemetryPacket packet) {
        telemetry.addData("Tag ID", d.id);
        try {
            double range = d.ftcPose.range;       // meters
            double bearing = d.ftcPose.bearing;   // deg
            double yaw = d.ftcPose.yaw, pitch = d.ftcPose.pitch, roll = d.ftcPose.roll;

            telemetry.addData("Range (m)", "%.3f", range);
            telemetry.addData("Bearing (deg)", "%.2f", bearing);
            telemetry.addData("Yaw/Pitch/Roll (deg)", "%.1f / %.1f / %.1f", yaw, pitch, roll);

            if (KNOWN_DISTANCE_M > 0) {
                double error = range - KNOWN_DISTANCE_M;
                telemetry.addData("Range error (m)", "%.3f (measured=%.3f)", error, KNOWN_DISTANCE_M);
                packet.put("range", range);
                packet.put("range_error", error);
            }
        } catch (Throwable t) {
            // Older SDKs may not have ftcPose; at least show pixel center
            try {
                telemetry.addData("Center (px)", "(%.1f, %.1f)", d.center.x, d.center.y);
                packet.put("center_x", d.center.x);
                packet.put("center_y", d.center.y);
            } catch (Throwable ignore) { }
        }
    }

    // -------------------- UI buttons --------------------

    private void handleButtons() {
        // A: toggle Driver Hub live-view (reflection-safe)
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev) {
            liveViewOn = !liveViewOn;
            if (liveViewOn) resumeLiveViewSafe();
            else stopLiveViewSafe();
        }
        aPrev = aNow;

        // Y: rebuild processor/portal with current intrinsics
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            telemetry.addLine("Rebuilding vision with current FX/FY/CX/CY...");
            telemetry.update();
            rebuildWithCurrentIntrinsics();
        }
        yPrev = yNow;
    }

    private void resumeLiveViewSafe() {
        callIfExists(visionPortal, "resumeLiveView", new Class[]{}, new Object[]{});
    }

    private void stopLiveViewSafe() {
        callIfExists(visionPortal, "stopLiveView", new Class[]{}, new Object[]{});
    }

    // -------------------- Reflection utility --------------------

    private static void callIfExists(Object target, String methodName, Class<?>[] paramTypes, Object... args) {
        if (target == null) return;
        try {
            Method m = target.getClass().getMethod(methodName, paramTypes);
            m.setAccessible(true);
            m.invoke(target, args);
        } catch (Exception ignored) {
            // Method not present on this SDK — safely skip.
        }
    }
}
