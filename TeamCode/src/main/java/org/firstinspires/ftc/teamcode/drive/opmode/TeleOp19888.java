package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp 19888")

public class TeleOp19888 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");
        DcMotor frontSlide = hardwareMap.dcMotor.get("frontSlide");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();



        int dir = 1;
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x;
            boolean clawOpen = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean clawClose = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean lowJunct = gamepad1.a || gamepad2.a;
            boolean midJunct = gamepad1.b || gamepad2.b;
            boolean hiJunct = gamepad1.y || gamepad2.y;
            boolean homing = gamepad1.x || gamepad2.x;
            boolean grab = gamepad1.x || gamepad2.x;
            boolean manualup = gamepad1.right_trigger >= 0.3 || gamepad2.right_trigger >= 0.3;
            boolean manualdown = gamepad1.left_trigger >= 0.3 || gamepad2.left_trigger >= 0.3;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower*limiter());
            leftRear.setPower(backLeftPower*limiter());
            rightFront.setPower(frontRightPower*limiter());
            rightRear.setPower(backRightPower*limiter());



        }
    }
    public double limiter(){
        if (gamepad1.left_bumper){
                return 1.0;}
        else return 0.6;
    }

    public int setSlideHeight(boolean lowToggle, boolean midToggle, boolean highToggle, boolean Home){

        if (Home){
            return 5;
        }
        else if (lowToggle){
            return 35;
        }
        else if (midToggle){
            return 60;
        }
        else if (highToggle){
            return 85;
        }
        return 0;
    }

}
