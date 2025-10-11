package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTag Orient (turn to face tag)")
public class AprilTagOrient extends LinearOpMode {

    // EXACT motor names
    private static final String LF = "leftFront";
    private static final String LB = "leftBack";
    private static final String RF = "rightFront";
    private static final String RB = "rightBack";

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Camera name in RC config
    private static final String WEBCAM_NAME = "Webcam 1";

    // Camera yaw calibration (deg). + means camera points LEFT of robot-forward.
    private double cameraYawOffsetDeg = 0.0;

    // Track any tag for orient, unless you set this to a specific ID (e.g., 1).
    private int focusTagId = -1;

    // >>> NEW: Stop-on-ID feature <<<
    private int stopTagId = 21;      // if this ID is detected, stop immediately
    private boolean stopLatched = false;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // PID on degrees
    private double kP = 0.045, kI = 0.000, kD = 0.0030;
    private double iSum = 0, lastErr = 0;

    private boolean orientEnabled = false;
    private boolean invertTurnSign = false; // press Y to flip if it runs away

    // Search + min torque
    private double minTurn = 0.18;
    private double sweepTurn = 0.22;
    private long   sweepMs   = 900;
    private long   lastSwap  = 0;
    private int    sweepDir  = +1;

    // Settle logic
    private double onTargetDeg = 1.5;
    private long   onTargetHoldMs = 250;
    private long   onTargetSince = 0;

    // Fade minTurn when close
    private double fadeStartDeg = 10.0;
    private double fadeEndDeg   = 2.5;

    // Bearing smoothing
    private Double lastBearingFilt = null;
    private double bearingAlpha = 0.65;

    // for telemetry
    private double lastLF, lastLB, lastRF, lastRB;

    @Override
    public void runOpMode() {
        // Motors
        leftFront  = hardwareMap.get(DcMotor.class, LF);
        leftBack   = hardwareMap.get(DcMotor.class, LB);
        rightFront = hardwareMap.get(DcMotor.class, RF);
        rightBack  = hardwareMap.get(DcMotor.class, RB);

        // Directions so +power drives forward (right side reversed)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(false); // set true if you want crisp stops

        // Vision
        tagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("A: ORIENT toggle | B: release | X: force spin | Y: invert turn");
        telemetry.addLine("D-pad L/R: trim camera offset  | LB/RB: slow modes");
        telemetry.addLine("Detects ID " + stopTagId + " -> STOP (press A or B to clear)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Toggles
            if (gamepad1.y) { invertTurnSign = !invertTurnSign; sleep(150); }
            if (gamepad1.a) { orientEnabled = !orientEnabled; sleep(150); resetPid(); onTargetSince = 0; stopLatched = false; }
            if (gamepad1.b) { orientEnabled = false; stopDrive(); stopLatched = false; }
            if (gamepad1.dpad_left)  { cameraYawOffsetDeg -= 0.5; sleep(120); }
            if (gamepad1.dpad_right) { cameraYawOffsetDeg += 0.5; sleep(120); }

            // Optional slow modes
            double speedScale = 1.0;
            if (gamepad1.left_bumper)       speedScale = 0.40;
            else if (gamepad1.right_bumper) speedScale = 0.30;

            double manualFwd  = -gamepad1.left_stick_y * speedScale;
            double manualTurn =  gamepad1.right_stick_x * speedScale;

            // FORCE SPIN diagnostic
            if (gamepad1.x && !stopLatched) {
                double t = 0.30; // spin power
                driveTurnWithForwardBias(invertTurnSign ? -t : t, 0.0);
                sendTelem(null, t, "FORCE SPIN", 0, manualFwd, manualTurn, false);
                continue;
            }

            // Get detections & check stop ID
            List<AprilTagDetection> dets = tagProcessor.getDetections();
            int detCount = (dets == null) ? 0 : dets.size();
            boolean sawStopId = containsId(dets, stopTagId);

            if (sawStopId) {
                stopLatched = true;
            }

            if (stopLatched) {
                // HARD STOP until driver clears with A or B
                stopDrive();
                sendTelem(null, 0.0, "STOP(ID " + stopTagId + ")", detCount, manualFwd, manualTurn, true);
                continue;
            }

            // Pick best tag (bearing in deg; left positive)
            Double tagBearingDegRaw = pickBearing(dets, focusTagId);

            // Low-pass the bearing to reduce jitter/overshoot
            Double tagBearingDeg = tagBearingDegRaw;
            if (tagBearingDegRaw != null) {
                lastBearingFilt = (lastBearingFilt == null)
                        ? tagBearingDegRaw
                        : (bearingAlpha * lastBearingFilt + (1 - bearingAlpha) * tagBearingDegRaw);
                tagBearingDeg = lastBearingFilt;
            } else {
                lastBearingFilt = null;
            }

            double turnCmd = 0;
            String modeInfo;

            if (orientEnabled) {
                if (tagBearingDeg != null) {
                    double errorDeg = (invertTurnSign ? -1 : 1) * (tagBearingDeg + cameraYawOffsetDeg);

                    // Deadband check for "on target"
                    boolean withinDeadband = Math.abs(errorDeg) <= onTargetDeg;
                    long now = System.currentTimeMillis();
                    if (withinDeadband) {
                        if (onTargetSince == 0) onTargetSince = now;
                    } else {
                        onTargetSince = 0;
                    }
                    boolean holdSatisfied = onTargetSince != 0 && (now - onTargetSince >= onTargetHoldMs);

                    if (holdSatisfied) {
                        // Fully on target: stop turning (no min torque)
                        resetPid();
                        driveTurnWithForwardBias(0.0, manualFwd * 0.15);
                        modeInfo = "LOCKED";
                    } else {
                        // PID (deg)
                        double p = kP * errorDeg;
                        iSum += errorDeg;
                        double d = kD * (errorDeg - lastErr);
                        lastErr = errorDeg;
                        double i = (kI == 0) ? 0 : kI * clamp(iSum, -200, 200);
                        turnCmd = p + i + d;

                        // Fade minTurn as we approach center
                        double absErr = Math.abs(errorDeg);
                        double fade = 1.0;
                        if (absErr <= fadeEndDeg) {
                            fade = 0.0;
                        } else if (absErr < fadeStartDeg) {
                            double t = (absErr - fadeEndDeg) / (fadeStartDeg - fadeEndDeg);
                            fade = clamp(t, 0.0, 1.0);
                        }
                        double minTorqueThisFrame = minTurn * fade;

                        // Apply min torque only if small output and not in deadband
                        if (!withinDeadband && Math.abs(turnCmd) < minTorqueThisFrame) {
                            turnCmd = Math.copySign(minTorqueThisFrame,
                                    (turnCmd == 0) ? errorDeg : turnCmd);
                        }

                        // Final clamp
                        turnCmd = clamp(turnCmd, -0.6, 0.6);

                        // Small forward bias to help keep tag in FOV
                        driveTurnWithForwardBias(turnCmd, manualFwd * 0.15);
                        modeInfo = withinDeadband ? "HOLDING" : "CORRECT";
                    }
                } else {
                    // No tag seen — sweep to find one
                    long now = System.currentTimeMillis();
                    if (now - lastSwap > sweepMs) { sweepDir = -sweepDir; lastSwap = now; }
                    turnCmd = clamp((invertTurnSign ? -1 : 1) * sweepDir * sweepTurn, -0.6, 0.6);
                    driveTurnWithForwardBias(turnCmd, 0.0);
                    modeInfo = "SWEEP";
                    onTargetSince = 0;
                }
            } else {
                // Manual arcade drive
                arcadeDrive(manualFwd, manualTurn);
                modeInfo = "MANUAL";
                onTargetSince = 0;
            }

            sendTelem(tagBearingDeg, turnCmd, modeInfo, detCount, manualFwd, manualTurn, false);
        }

        if (visionPortal != null) visionPortal.close();
    }

    /** Returns true if any detection has the given ID. */
    private static boolean containsId(List<AprilTagDetection> dets, int id) {
        if (dets == null) return false;
        for (AprilTagDetection d : dets) {
            if (d.id == id) return true;
        }
        return false;
    }

    /** Pick tag with smallest |bearing|; tie-break nearest range (SDK-friendly). */
    private static Double pickBearing(List<AprilTagDetection> dets, int onlyId) {
        if (dets == null || dets.isEmpty()) return null;
        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if (d.ftcPose == null) continue;
            if (onlyId >= 0 && d.id != onlyId) continue;
            if (best == null) { best = d; continue; }
            double bBest = Math.abs(best.ftcPose.bearing);
            double bCur  = Math.abs(d.ftcPose.bearing);
            if (bCur < bBest - 1e-6) best = d;
            else if (Math.abs(bCur - bBest) <= 1e-6 && d.ftcPose.range < best.ftcPose.range) best = d;
        }
        return (best == null || best.ftcPose == null) ? null : best.ftcPose.bearing;
    }

    private void resetPid() { iSum = 0; lastErr = 0; }

    private void stopDrive() { setPowers(0,0,0,0); }

    private void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior z = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFront.setZeroPowerBehavior(z);
        rightFront.setZeroPowerBehavior(z);
        leftBack.setZeroPowerBehavior(z);
        rightBack.setZeroPowerBehavior(z);
    }

    private void driveTurnWithForwardBias(double turn, double fwd) {
        double lf =  fwd + turn, rf =  fwd - turn, lb =  fwd + turn, rb =  fwd - turn;
        normalizeAndSet(lf, rf, lb, rb);
    }

    private void arcadeDrive(double fwd, double turn) {
        double lf =  fwd + turn, rf =  fwd - turn, lb =  fwd + turn, rb =  fwd - turn;
        normalizeAndSet(lf, rf, lb, rb);
    }

    private void normalizeAndSet(double lf, double rf, double lb, double rb) {
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        setPowers(lf/max, rf/max, lb/max, rb/max);
    }

    private void setPowers(double lf, double rf, double lb, double rb) {
        lastLF = lf; lastRF = rf; lastLB = lb; lastRB = rb;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private void sendTelem(Double bearingDeg, double turnCmd, String mode, int detCount,
                           double fwd, double turnStick, boolean stoppedById) {
        telemetry.addData("Mode", mode + (orientEnabled ? " (ORIENT ON)" : " (ORIENT OFF)"));
        telemetry.addData("Camera", String.valueOf(visionPortal.getCameraState()));
        telemetry.addData("Detections", detCount);      
        telemetry.addData("Bearing (deg)", bearingDeg == null ? "none" : String.format("%.2f", bearingDeg));
        telemetry.addData("Turn cmd", "%.2f", turnCmd);
        telemetry.addData("Offset trim (D-pad L/R)", "%.1f°", cameraYawOffsetDeg);
        telemetry.addData("Invert turn (Y)", invertTurnSign ? "ON" : "OFF");
        telemetry.addData("Motor LF/RF/LB/RB", "%.2f / %.2f / %.2f / %.2f", lastLF, lastRF, lastLB, lastRB);
        if (stoppedById) {
            telemetry.addLine(">>> STOP LATCHED: Saw Tag ID " + stopTagId + " (press A or B to clear) <<<");
        }
        telemetry.update();
    }
}
