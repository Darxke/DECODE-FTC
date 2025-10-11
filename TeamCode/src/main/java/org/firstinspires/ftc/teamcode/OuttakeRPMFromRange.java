package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="OuttakeRPMFromRange (Distance/Height→RPM)")
public class OuttakeRPMFromRange extends LinearOpMode {

    private double outtakeGear_G        = 1.0;
    private double outtakeWheelRadius_m = 0.04445;
    private double outtakeSlipEta       = 0.80;
    private double outtakeHoodDeg       = 70.0; // from horizontal
    private double outtakeExitHeight_m  = 0.70;
    private double outtakeGoalHeight_m  = 1.05;

    private double distance_m   = 3.00;
    private double stepDist_m   = 0.10;
    private double heightStep_m = 0.02;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up)   { distance_m += stepDist_m; sleep(120); }
            if (gamepad1.dpad_down) { distance_m = Math.max(0.10, distance_m - stepDist_m); sleep(120); }
            if (gamepad1.left_bumper)  { outtakeGoalHeight_m = Math.max(0, outtakeGoalHeight_m - heightStep_m); sleep(120); }
            if (gamepad1.right_bumper) { outtakeGoalHeight_m += heightStep_m; sleep(120); }

            double theta = Math.toRadians(outtakeHoodDeg);
            double h     = outtakeGoalHeight_m - outtakeExitHeight_m;

            double v = Ballistics.speedForRange(distance_m, theta, h); // alias to ...NoDrag

            String motorRPMText, vText, wheelRPMText;
            if (Double.isNaN(v)) {
                motorRPMText = "impossible (raise angle or lower target height)";
                vText = wheelRPMText = "-";
            } else {
                double wheelRPM   = (60.0 / (2.0 * Math.PI)) * (v / (outtakeSlipEta * outtakeWheelRadius_m));
                double motorRPM   = wheelRPM * outtakeGear_G;
                motorRPMText      = String.format("%.0f", motorRPM);
                vText             = String.format("%.2f", v);
                wheelRPMText      = String.format("%.0f", wheelRPM);
            }

            telemetry.addLine("=== Outtake RPM From Range (Distance/Height→RPM) ===");
            telemetry.addData("Distance (m)  (D-Pad ±)", String.format("%.2f", distance_m));
            telemetry.addData("Goal height (m) (LB/RB ±)", String.format("%.2f", outtakeGoalHeight_m));
            telemetry.addData("Required Exit v (m/s)", vText);
            telemetry.addData("Required Motor RPM", motorRPMText);
            telemetry.addData("Wheel RPM", wheelRPMText);
            telemetry.update();
        }
    }
}
