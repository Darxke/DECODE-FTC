package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="OuttakeRangeEstimator (RPM→Distance)")
public class OuttakeRangeEstimator extends LinearOpMode {

    private double outtakeGear_G        = 1.0;
    private double outtakeWheelRadius_m = 0.04445;
    private double outtakeSlipEta       = 0.80;
    private double outtakeHoodDeg       = 70.0; // from horizontal
    private double outtakeExitHeight_m  = 0.70;
    private double outtakeGoalHeight_m  = 1.05;

    private double outtakeMotorRPM = 3500;
    private double stepRPM         = 200;
    private double heightStep_m    = 0.02;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up)   { outtakeMotorRPM += stepRPM; sleep(120); }
            if (gamepad1.dpad_down) { outtakeMotorRPM = Math.max(0, outtakeMotorRPM - stepRPM); sleep(120); }
            if (gamepad1.left_bumper)  { outtakeGoalHeight_m = Math.max(0, outtakeGoalHeight_m - heightStep_m); sleep(120); }
            if (gamepad1.right_bumper) { outtakeGoalHeight_m += heightStep_m; sleep(120); }

            double dMeters = Ballistics.distanceFromMotorRPM(
                    outtakeMotorRPM, outtakeGear_G, outtakeWheelRadius_m, outtakeSlipEta,
                    outtakeHoodDeg, outtakeExitHeight_m, outtakeGoalHeight_m
            );

            double wheelRPM = Ballistics.wheelRPM(outtakeMotorRPM, outtakeGear_G);
            double v        = Ballistics.exitSpeed(wheelRPM, outtakeWheelRadius_m, outtakeSlipEta);

            telemetry.addLine("=== Outtake Range Estimator (RPM→Distance) ===");
            telemetry.addData("Motor RPM (D-Pad ±)", Math.round(outtakeMotorRPM));
            telemetry.addData("Exit v (m/s)", String.format("%.2f", v));
            telemetry.addData("Goal height (m) (LB/RB ±)", String.format("%.2f", outtakeGoalHeight_m));
            telemetry.addData("Predicted distance (m)",
                    Double.isNaN(dMeters) ? "impossible (raise RPM/angle)" : String.format("%.2f", dMeters));
            telemetry.update();
        }
    }
}
