package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Method;
import java.util.List;

@Autonomous(name = "AutoApril (Tag → Action)", group = "Auto")
public class AutoApril extends LinearOpMode {

    // --- Camera name in RC config ---
    private static final String WEBCAM_NAME = "Webcam 1";

    // --- Your tag IDs ---
    private static final int ID_20 = 20; // blue
    private static final int ID_21 = 21; // gpp
    private static final int ID_22 = 22; // pgp
    private static final int ID_23 = 23; // ppg
    private static final int ID_24 = 24; // red

    // If +power drives backward on your robot, set FWD = -1.0
    private static final double FWD = 1.0;

    // After Start, how long to keep scanning for a tag (ms). Set 0 to wait forever.
    private static final long TIMEOUT_MS = 4000;

    // --- Drive motors ---
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        // Init hardware
        initDrive();
        // Init vision
        initVision();

        // ===== INIT: scan & snapshot =====
        while (opModeInInit()) {
            tagOfInterest = latestTarget();
            if (tagOfInterest != null) {
                telemetry.addData("TAG SEEN", "ID=%d", tagOfInterest.id);
                addPoseIfAvailable(tagOfInterest);
            } else {
                telemetry.addLine("No target tag yet…");
            }
            telemetry.addLine("Press ▶ when ready.");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        // ===== AFTER START: keep scanning until tag or timeout =====
        long start = System.currentTimeMillis();
        while (opModeIsActive() && tagOfInterest == null &&
                (TIMEOUT_MS == 0 || System.currentTimeMillis() - start < TIMEOUT_MS)) {
            tagOfInterest = latestTarget();
            telemetry.addLine("Scanning for tag after Start...");
            if (tagOfInterest != null) telemetry.addData("FOUND Tag ID", tagOfInterest.id);
            telemetry.update();
            sleep(20);
        }
        int chosen = (tagOfInterest != null) ? tagOfInterest.id : -1;
        telemetry.addData("Chosen Tag", chosen);
        telemetry.update();

        // Optional: stop overlays/preview for max performance
        setProcessorEnabledSafe(aprilTag, false);
        stopLiveViewSafe();

        // ===== ACTIONS BY TAG ID =====
        if (chosen == ID_20) {
            telemetry.addLine("Branch: ID 20");
            telemetry.update();
            driveForward(0.6, 900);
            turnRight(0.5, 550);
            parkStop();

        } else if (chosen == ID_21) {
            telemetry.addLine("Branch: ID 21");
            telemetry.update();
            strafeLeft(0.6, 800);
            driveForward(0.6, 700);
            parkStop();

        } else if (chosen == ID_22) {
            telemetry.addLine("Branch: ID 22");
            telemetry.update();
            strafeRight(0.6, 800);
            driveForward(0.6, 700);
            parkStop();

        } else if (chosen == ID_23) {
            telemetry.addLine("Branch: ID 23");
            telemetry.update();
            driveForward(0.6, 500);
            turnLeft(0.5, 550);
            driveForward(0.6, 600);
            parkStop();

        } else if (chosen == ID_24) {
            telemetry.addLine("Branch: ID 24");
            telemetry.update();
            driveForward(-0.5, 600); // back up
            strafeRight(0.6, 900);
            parkStop();

        } else {
            // Fallback if nothing was seen
            telemetry.addLine("No tag detected — running fallback.");
            telemetry.update();
            driveForward(0.4, 600);
            parkStop();
        }
    }

    // ---------------- Drive init ----------------
    private void initDrive() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match your TeleOp: left side reversed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    // ---------------- Vision init ----------------
    private void initVision() {
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        // (Optional) setTagSize / setLensIntrinsics can be added later if your SDK supports them
        aprilTag = builder.build();
        callIfExists(aprilTag, "setDecimation", new Class[]{float.class}, 2.0f);

        VisionPortal.Builder vp = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag);
        callIfExists(vp, "enableLiveView", new Class[]{boolean.class}, true);
        callIfExists(vp, "setAutoStopLiveView", new Class[]{boolean.class}, false);

        visionPortal = vp.build();
    }

    // ---------------- Detection helpers ----------------
    /** Return the most recent detection among the IDs we care about. */
    private AprilTagDetection latestTarget() {
        List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection pick = null;
        for (AprilTagDetection d : dets) {
            if (d.id == ID_20 || d.id == ID_21 || d.id == ID_22 || d.id == ID_23 || d.id == ID_24) {
                pick = d; // keep last matching detection this frame
            }
        }
        return pick;
    }

    private void addPoseIfAvailable(AprilTagDetection d) {
        try {
            telemetry.addData("Range(m)",   "%.3f", d.ftcPose.range);
            telemetry.addData("Bearing(°)", "%.2f", d.ftcPose.bearing);
        } catch (Throwable ignore) {
            // older SDKs may not expose ftcPose
        }
    }

    // ---------------- Simple drive helpers (timed) ----------------
    private void driveForward(double power, long ms) {
        double p = FWD * power; // global flip if needed
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
        sleep(ms);
        stopDrive();
    }

    private void turnLeft(double power, long ms) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        sleep(ms);
        stopDrive();
    }

    private void turnRight(double power, long ms) {
        turnLeft(-power, ms);
    }

    private void strafeLeft(double power, long ms) {
        // Mecanum strafe: LF -power, LB +power, RF +power, RB -power
        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
        sleep(ms);
        stopDrive();
    }

    private void strafeRight(double power, long ms) {
        strafeLeft(-power, ms);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void parkStop() {
        stopDrive();
        telemetry.addLine("Done.");
        telemetry.update();
        sleep(250);
    }

    // ---------------- Vision safe calls ----------------
    private void stopLiveViewSafe() {
        callIfExists(visionPortal, "stopLiveView", new Class[]{}, new Object[]{});
    }

    private void setProcessorEnabledSafe(Object processor, boolean enabled) {
        callIfExists(visionPortal, "setProcessorEnabled",
                new Class[]{Object.class, boolean.class}, processor, enabled);
    }

    private static void callIfExists(Object target, String method, Class<?>[] paramTypes, Object... args) {
        if (target == null) return;
        try {
            Method m = target.getClass().getMethod(method, paramTypes);
            m.setAccessible(true);
            m.invoke(target, args);
        } catch (Exception ignored) { }
    }
}
