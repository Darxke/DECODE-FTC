package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTag Center-Track (No Tag Size, Any 36h11)", group="teleop")
public class AprilTagTrack extends LinearOpMode {

    // ===== Camera & centering settings =====
    // We explicitly set camera resolution to 1280x720 so pixel math matches.
    private static final int    IMG_WIDTH   = 1280; // must match setCameraResolution width
    private static final double HFOV_DEG    = 70.0; // approx webcam horizontal FOV; tune if needed
    // If camera is not exactly centered on robot, a tiny bias helps the "robot center" face the tag
    private static final double ROBOT_CENTER_OFFSET_DEG = 0.0;

    // (Optional) real camera mounting, only used if ftcPose ever becomes available
    private static final double CAM_X_IN = 4.0;   // +X right of robot center, inches
    private static final double CAM_Z_IN = 6.0;   // +Z forward of rohbot center, inches
    private static final double CAM_YAW_DEG = 0.0;// + yaw turns right

    private static final double MAX_ROT_POWER = 0.5;
    private final PID headingPid = new PID(0.010, 0.0002, 0.002);

    // DRIVE (use your exact config names)
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // VISION
    private AprilTagProcessor tagProc;
    private VisionPortal portal;

    @Override
    public void runOpMode() {
        // ===== MOTORS =====
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Reversed wheels (per your request)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront,rightFront,leftBack,rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ===== VISION: build processor now; start camera on PLAY =====
        try {
            tagProc = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)  
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    // Output units for ftcPose (if it ever exists). We don't rely on pose here.
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    // Detect ANY 36h11 tag. We do NOT set tag size.
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            telemetry.addLine("Vision ready (camera starts on PLAY).");
        } catch (Exception e) {
            tagProc = null;
            telemetry.addLine("Vision init failed; driving only.");
        }
        telemetry.update();

        // ===== WAIT FOR PLAY =====
        waitForStart();

        // Auto-start camera when match starts
        ensureVisionStreaming();

        long lastT = System.nanoTime();
        headingPid.reset();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastT) / 1e9;
            lastT = now;

            // Driver translation
            double forwardCmd = -gamepad1.left_stick_y * 0.7; // +forward
            double strafeCmd  =  gamepad1.left_stick_x * 0.7; // +right

            // Compute heading error from pixel-centering (no size needed).
            // If ftcPose happens to be available (e.g., you later add size), we'll auto-use robot-center math instead.
            Double headingErrDeg = getHeadingErrorDegAnyMode();

            double turnCmd;
            if (headingErrDeg != null) {
                double pidOut = headingPid.calculate(headingErrDeg, dt);
                turnCmd = clamp(pidOut, -MAX_ROT_POWER, MAX_ROT_POWER);
                telemetry.addData("Tag", "seen");
                telemetry.addData("headingErr", "%.2f deg", headingErrDeg);
            } else {
                telemetry.addData("Tag", "not seen");
                headingPid.reset();
                // manual rotate when no tag:
                turnCmd = gamepad1.right_stick_x * 0.6;
            }

            telemetry.addData("CameraState", (portal == null) ? "null" : portal.getCameraState());
            setDrive(forwardCmd, strafeCmd, turnCmd);
            telemetry.update();

            // Optional: A pauses, B resumes camera mid-match
            if (gamepad1.a && portal != null) portal.stopStreaming();
            if (gamepad1.b && portal != null) portal.resumeStreaming();
        }

        setDrive(0,0,0);
        if (portal != null) portal.close();
    }

    // === Auto start/resume streaming on PLAY, force 1280x720 so IMG_WIDTH matches ===
    private void ensureVisionStreaming() {
        if (tagProc == null) return;
        if (portal == null) {
            WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1"); // rename if needed
            portal = new VisionPortal.Builder()
                    .setCamera(cam)
                    .setCameraResolution(new Size(IMG_WIDTH, 720)) // ensure width=IMG_WIDTH
                    .addProcessor(tagProc)
                    .build();
        } else {
            portal.resumeStreaming();
        }
        long deadline = System.currentTimeMillis() + 1000;
        while (!isStopRequested()
                && System.currentTimeMillis() < deadline
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
    }

    // ========================= Heading calculation =========================
    // Uses ftcPose if available (requires tag size; gives true robot-center bearing).
    // Otherwise, uses pixel-centering: angle ~ horizontal pixel offset mapped via HFOV.
    private Double getHeadingErrorDegAnyMode() {
        if (tagProc == null || portal == null
                || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return null;

        List<AprilTagDetection> dets = tagProc.getDetections();
        int n = (dets == null) ? 0 : dets.size();
        telemetry.addData("#detections", n);
        if (n == 0) return null;

        // Pick detection whose center is closest to image center horizontally
        AprilTagDetection best = null;
        double bestDx = Double.POSITIVE_INFINITY;
        for (AprilTagDetection d : dets) {
            double dx = Math.abs(d.center.x - IMG_WIDTH * 0.5);
            if (dx < bestDx) { bestDx = dx; best = d; }
            telemetry.addData("id", d.id);
        }
        if (best == null) return null;

        // If ftcPose exists (e.g., you later add tag size), prefer robot-center math
        if (best.ftcPose != null) {
            double tx_right   = -best.ftcPose.y;  // FTC pose y=LEFT(+)
            double tz_forward =  best.ftcPose.x;  // FTC pose x=FORWARD(+)
            telemetry.addData("mode", "pose");
            return robotCenterHeadingErrorDeg(tx_right, tz_forward, CAM_X_IN, CAM_Z_IN, CAM_YAW_DEG);
        }

        // Pixel-centering fallback (no tag size needed)
        telemetry.addData("mode", "pixel");
        double pxErr = (best.center.x - IMG_WIDTH * 0.5); // pixels, + right
        double norm  = pxErr / (IMG_WIDTH * 0.5);         // [-1..1]
        double angleDeg = norm * (HFOV_DEG * 0.5);        // degrees, + right
        return angleDeg + ROBOT_CENTER_OFFSET_DEG;
    }

    // True robot-center bearing (used only if we have real pose)
    private static double robotCenterHeadingErrorDeg(
            double txCamRight, double tzCamForward,
            double camX_in, double camZ_in, double camYaw_deg) {

        double yaw = Math.toRadians(camYaw_deg);
        double cos = Math.cos(yaw), sin = Math.sin(yaw);

        // rotate camera->tag vector into robot frame (yaw only)
        double vx_r_from_cam =  cos * txCamRight + sin * tzCamForward;
        double vz_r_from_cam = -sin * txCamRight + cos * tzCamForward;

        // shift by camera position to get ROBOT-CENTER -> tag (in robot frame)
        double vx_r = camX_in + vx_r_from_cam; // right (+)
        double vz_r = camZ_in + vz_r_from_cam; // forward (+)

        return Math.toDegrees(Math.atan2(vx_r, vz_r)); // + means tag to the right
    }

    // ========================= Drive =========================
    private void setDrive(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        lf /= max; rf /= max; lb /= max; rb /= max;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    // ========================= Utils =========================
    private static class PID {
        double kP, kI, kD, integ = 0, prevErr = 0, integLimit = 1.0;
        PID(double p, double i, double d){kP=p; kI=i; kD=d;}
        void reset(){integ=0; prevErr=0;}
        double calculate(double error, double dt){
            integ += error * dt;
            if (integ >  integLimit) integ =  integLimit;
            if (integ < -integLimit) integ = -integLimit;
            double deriv = (error - prevErr) / Math.max(dt, 1e-6);
            prevErr = error;
            return kP*error + kI*integ + kD*deriv;
        }
    }
    private static double clamp(double v,double lo,double hi){ return Math.max(lo, Math.min(hi, v)); }
}
