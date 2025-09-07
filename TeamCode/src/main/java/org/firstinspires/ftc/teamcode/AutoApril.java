package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Auto", group = "Auto")
public class AutoApril extends LinearOpMode {

    // === Configure these for YOUR field layout ===
    // Common FTC use: LEFT=1, MIDDLE=2, RIGHT=3  (change if your IDs differ)
    public static final int ID_LEFT = 1;
    public static final int ID_MIDDLE = 2;
    public static final int ID_RIGHT = 3;

    // Tag size in METERS (0.166 m = 16.6 cm)
    public static final double TAG_SIZE_METERS = 0.166;

    // Name of your webcam in the RC configuration
    public static final String WEBCAM_NAME = "Webcam 1";

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Snapshot of the tag seen during INIT (saved for use after Start)
    private AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        initVision();

        // INIT loop: keep looking for tags and remember the last good one
        while (opModeInInit()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            // Filter to the IDs we care about
            tagOfInterest = null;
            for (AprilTagDetection det : detections) {
                if (det.id == ID_LEFT || det.id == ID_MIDDLE || det.id == ID_RIGHT) {
                    tagOfInterest = det; // keep the last one seen this loop
                }
            }

            // Telemetry
            if (tagOfInterest != null) {
                telemetry.addLine("TAG IN VIEW (snapshot will be saved)");
                showTagTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("No target tag in view yet...");
            }
            telemetry.addLine("Press ▶ to start; snapshot will be used.");
            telemetry.update();

            sleep(20);
        }

        // Start pressed
        if (isStopRequested()) return;

        // Freeze the snapshot (tagOfInterest already holds the last seen one)
        int chosen = (tagOfInterest != null) ? tagOfInterest.id : -1;

        // Optional: toggle camera off to save CPU after detection
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.stopStreaming();
        }

        // === Branch your auto ===
        if (chosen == ID_LEFT) {
            runLeftAuto();
        } else if (chosen == ID_MIDDLE) {
            runMiddleAuto();
        } else if (chosen == ID_RIGHT) {
            runRightAuto();
        } else {
            runFallbackAuto(); // nothing seen: safe default/park
        }
    }

    // ------------------- Helpers -------------------

    private void initVision() {
        // Build AprilTag processor
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);

        // If you have calibrated intrinsics, you can set them here (optional):
        // tagBuilder.setLensIntrinsics(fx, fy, cx, cy);

        aprilTag = tagBuilder.build();

        // Build VisionPortal using a USB webcam (recommended for stability)
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();

        // Optional tuning: lower decimation increases range (but costs FPS)
        aprilTag.setDecimation(2); // 1–3 is typical; 3 = fast/close, 1 = slow/far
    }

    private void showTagTelemetry(AprilTagDetection det) {
        telemetry.addData("ID", det.id);
        telemetry.addData("Center (px)", "(%.1f, %.1f)", det.center.x, det.center.y);
        telemetry.addData("Range (m)", "%.2f", det.ftcPose.range);
        telemetry.addData("Bearing (deg)", "%.1f", det.ftcPose.bearing);
        telemetry.addData("Yaw/Pitch/Roll (deg)", "%.1f / %.1f / %.1f",
                det.ftcPose.yaw, det.ftcPose.pitch, det.ftcPose.roll);
    }

    // ====== Replace these with YOUR drivetrain/arm actions ======
    private void runLeftAuto() {
        telemetry.addLine("Running LEFT auto (ID=" + ID_LEFT + ")");
        telemetry.update();

        // EXAMPLE (replace with RoadRunner or your drive code)
        // drive.forward(30);
        // turn(90);
        // placePixel();
        park();
    }

    private void runMiddleAuto() {
        telemetry.addLine("Running MIDDLE auto (ID=" + ID_MIDDLE + ")");
        telemetry.update();

        // drive.forward(36);
        // placePixel();
        park();
    }

    private void runRightAuto() {
        telemetry.addLine("Running RIGHT auto (ID=" + ID_RIGHT + ")");
        telemetry.update();

        // drive.forward(24);
        // strafeRight(20);
        // placePixel();
        park();
    }

    private void runFallbackAuto() {
        telemetry.addLine("No tag snapshot — running FALLBACK/SAFE auto");
        telemetry.update();

        // Minimal movement / safe park
        park();
    }

    private void park() {
        // drive.forward(6);
        telemetry.addLine("Parked.");
        telemetry.update();
        sleep(250);
    }
}
