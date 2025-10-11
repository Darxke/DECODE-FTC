package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OuttakeDistanceTest (PID + MeasuredRPM + Drag Toggle)")
public class OuttakeDistanceTest extends LinearOpMode {

    private static final String OUTTAKE_MOTOR_NAME = "outtake";

    // Geometry / physics (edit as needed)
    private double outtakeGear_G        = 1.0;        // 1:1
    private double outtakeWheelRadius_m = 0.04445;    // 1.75 in
    private double outtakeSlipEta       = 0.80;       // start realistic (0.70–0.85)
    private double outtakeHoodDeg       = 70.0;       // NOTE: from HORIZONTAL. If your 70° is from VERTICAL, use 20.0
    private double outtakeExitHeight_m  = 0.70;
    private double outtakeGoalHeight_m  = 1.05;

    // Projectile (5 in)
    private double projDiameter_m = 0.127;
    private double projMass_kg    = 0.060;            // update if known
    private double projCd         = 0.80;             // tune 0.7–1.0

    // Control
    private double targetMotorRPM = 3500, rpmStep = 200;

    // Encoder: your datasheet shows 28 PPR, 1:1 gearbox, quadrature 4× -> 112 ticks/rev
    private double ticksPerRev = 112.0;

    // PID
    private double kP = 0.0008, kI = 0.0, kD = 0.0002;

    // UI / gating
    private double readyBandFrac = 0.05, readyHold_s = 0.25, heightStep_m = 0.02;
    private boolean dragMode = false;                 // B to toggle
    private boolean useMeasuredForPrediction = true;  // Y to toggle

    // Simple “fit η” helper
    private double measuredDistance_m = 2.00, distStep_m = 0.10;

    private DcMotorEx outtakeMotor;
    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean outtakeEnabled = false;

    static class PID {
        double kP,kI,kD,sum=0,lastError=0,lastT=0,sumClamp=10_000;
        PID(double p,double i,double d){kP=p;kI=i;kD=d;}
        double update(double target,double measured,double t){
            double e=target-measured; double dt=Math.max(t-lastT,1e-3);
            sum+=e*dt; if(sum>sumClamp)sum=sumClamp; if(sum<-sumClamp)sum=-sumClamp;
            double dErr=(e-lastError)/dt; lastError=e; lastT=t;
            return kP*e + kI*sum + kD*dErr;
        }
        void reset(){sum=0; lastError=0; lastT=0;}
    }

    @Override
    public void runOpMode() {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, OUTTAKE_MOTOR_NAME);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // as requested
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PID pid = new PID(kP,kI,kD);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        loopTimer.reset(); readyTimer.reset(); pid.reset();

        while (opModeIsActive()) {
            double now = loopTimer.seconds();

            // Controls
            if (gamepad1.a) { outtakeEnabled = !outtakeEnabled; sleep(180); pid.reset(); readyTimer.reset(); }
            if (gamepad1.dpad_up)   { targetMotorRPM += rpmStep; sleep(120); }
            if (gamepad1.dpad_down) { targetMotorRPM = Math.max(0, targetMotorRPM - rpmStep); sleep(120); }
            if (gamepad1.left_bumper)  { outtakeGoalHeight_m = Math.max(0, outtakeGoalHeight_m - heightStep_m); sleep(120); }
            if (gamepad1.right_bumper) { outtakeGoalHeight_m += heightStep_m; sleep(120); }
            if (gamepad1.b) { dragMode = !dragMode; sleep(180); }
            if (gamepad1.y) { useMeasuredForPrediction = !useMeasuredForPrediction; sleep(180); }
            if (gamepad1.dpad_left)  { measuredDistance_m = Math.max(0.50, measuredDistance_m - distStep_m); sleep(120); }
            if (gamepad1.dpad_right) { measuredDistance_m += distStep_m; sleep(120); }

            double targetTPS = (targetMotorRPM * ticksPerRev) / 60.0;
            double measTPS   = Math.abs(outtakeMotor.getVelocity());
            double measRPM   = measTPS * 60.0 / Math.max(1e-6, ticksPerRev);

            // PID -> power
            double powerCmd;
            if (outtakeEnabled) powerCmd = Range.clip(pid.update(targetTPS, measTPS, now), 0, 1);
            else { pid.reset(); powerCmd = 0; }
            outtakeMotor.setPower(powerCmd);

            // Ready gate
            boolean inBand = Math.abs(targetTPS - measTPS) <= readyBandFrac * Math.max(50.0, targetTPS);
            if (!inBand) readyTimer.reset();
            boolean ready = inBand && (readyTimer.seconds() >= readyHold_s);
            if (inBand && readyTimer.seconds()==0) readyTimer.reset();

            // Choose RPM for prediction
            double rpmForPrediction = useMeasuredForPrediction ? measRPM : targetMotorRPM;

            // Predictions
            double dNoDrag = Ballistics.distanceFromMotorRPM(
                    rpmForPrediction, outtakeGear_G, outtakeWheelRadius_m, outtakeSlipEta,
                    outtakeHoodDeg, outtakeExitHeight_m, outtakeGoalHeight_m
            );
            double dDrag = Ballistics.distanceFromMotorRPMWithDrag(
                    rpmForPrediction, outtakeGear_G, outtakeWheelRadius_m, outtakeSlipEta,
                    outtakeHoodDeg, outtakeExitHeight_m, outtakeGoalHeight_m,
                    projMass_kg, projDiameter_m, projCd
            );
            double shown = dragMode ? dDrag : dNoDrag;

            // Quick η fitter (press X after you observe a real shot distance)
            if (gamepad1.x) {
                double theta = Math.toRadians(outtakeHoodDeg);
                double h     = outtakeGoalHeight_m - outtakeExitHeight_m;

                double vReq;
                if (dragMode) {
                    double lo = 1.0, hi = 50.0;
                    for (int it=0; it<40; it++) {
                        double mid = 0.5*(lo+hi);
                        double dMid = Ballistics.rangeWithDrag(mid, theta, outtakeExitHeight_m, outtakeGoalHeight_m,
                                projMass_kg, projDiameter_m, projCd);
                        if (Double.isNaN(dMid) || dMid < measuredDistance_m) lo = mid; else hi = mid;
                    }
                    vReq = 0.5*(lo+hi);
                } else {
                    vReq = Ballistics.speedForRangeNoDrag(measuredDistance_m, theta, h);
                }

                double wheelRPM = Ballistics.wheelRPM(rpmForPrediction, outtakeGear_G);
                double omega = (2.0*Math.PI/60.0) * wheelRPM;
                double etaFit = vReq / (omega * outtakeWheelRadius_m);
                outtakeSlipEta = Range.clip(etaFit, 0.4, 0.98);
            }

            telemetry.addLine("=== Outtake Distance TEST ===");
            telemetry.addData("Enabled (A)", outtakeEnabled ? "ON" : "OFF");
            telemetry.addData("Motor RPM tgt/meas", "%d / %d", Math.round(targetMotorRPM), Math.round(measRPM));
            telemetry.addData("Predict w/", useMeasuredForPrediction ? "MEASURED RPM (Y toggle)" : "TARGET RPM (Y toggle)");
            telemetry.addData("Predicted distance (m)", Double.isNaN(shown) ? "impossible" : String.format("%.2f", shown));
            telemetry.addData("Model", dragMode ? "DRAG (B toggle)" : "NO-DRAG (B toggle)");
            telemetry.addData("Goal height (m) (LB/RB ±)", String.format("%.2f", outtakeGoalHeight_m));
            telemetry.addData("Slip η", String.format("%.3f", outtakeSlipEta));
            telemetry.addData("Fit distance (L/R, X to fit η)", String.format("%.2f m", measuredDistance_m));
            telemetry.addData("measTPS / measRPM", "%.0f / %.0f", measTPS, measRPM);
            telemetry.addData("Drag params", "d=%.0f mm, m=%.0f g, Cd=%.2f",
                    projDiameter_m*1000, projMass_kg*1000, projCd);
            telemetry.update();
        }
    }
}
