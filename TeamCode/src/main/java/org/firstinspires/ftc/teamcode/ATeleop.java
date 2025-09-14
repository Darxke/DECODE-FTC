package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ATeleop")
public class ATeleop extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeL;
    private DcMotor intakeR;
    private DcMotor outtake;

    private CRServo intakeM; // CRServo, not Servo
    private CRServo outtakeL;
    private CRServo outtakeR;

    @Override
    public void runOpMode()  {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intakeM = hardwareMap.get(CRServo.class, "intakeM");
        outtakeR = hardwareMap.get(CRServo.class, "outtakeR"); // CRServo mapping
        outtakeL = hardwareMap.get(CRServo.class, "outtakeL"); // CRServo mapping

        // CRServo mapping

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

                float RFspeed = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                float RBspeed = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                double maxSpeed = 1;
                if (gamepad1.left_bumper) {
                    maxSpeed = 0.4;
                } else if (gamepad1.right_bumper) {
                    maxSpeed = 0.3;
                }
                LFspeed = (float) Range.clip(LFspeed, -maxSpeed, maxSpeed);
                LBspeed = (float) Range.clip(LBspeed, -maxSpeed, maxSpeed);
                RFspeed = (float) Range.clip(RFspeed, -maxSpeed, maxSpeed);
                RBspeed = (float) Range.clip(RBspeed, -maxSpeed, maxSpeed);
                leftFront.setPower(LFspeed);
                leftBack.setPower(LBspeed);
                rightFront.setPower(RFspeed);
                rightBack.setPower(RBspeed);

                // ================== TOGGLES ==================
                // Right bumper: toggle both intakes + CRServo
                if (gamepad2.right_bumper) {
                    if (intakeL.getPower() == 0 && intakeR.getPower() == 0) {
                        intakeL.setPower(1.0);
                        intakeR.setPower(1.0);
                        intakeM.setPower(1.0);// CRServo forward
                    } else {
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        intakeM.setPower(0); // CRServo stop
                    }
                    while (opModeIsActive() && gamepad2.right_bumper) { idle(); }
                }

                // B button: toggle outtake
                if (gamepad2.b){
                    if (outtake.getPower() == 0) {
                        outtake.setPower(1.0);
                        outtakeL.setPower(1);
                        outtakeR.setPower(1);
                    } else {
                        outtake.setPower(0);
                        outtakeL.setPower(0);
                        outtakeR.setPower(0);
                    }
                    while (opModeIsActive() && gamepad2.b) { idle(); }
                }
                // ==============================================
            }
        }
    }
}
