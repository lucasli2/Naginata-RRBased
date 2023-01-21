package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot19888;


@TeleOp(name = "Andrew Tele")

public class AndrewTele extends LinearOpMode {

    double y;// Remember, this is reversed!
    double x;// Counteract imperfect strafing
    double rx;
    double dz=0.1;
    boolean low, mid, high, zero, tare, liftUp_M, liftDw_M, claw_open, claw_close;



    // Declare our motors
    // Make sure your ID's match your configuration
    public double ret(double x) {
        if ((x<=dz) && (x>=(-1.0*dz))) return 0.0;

        if(x>0.0) {
            x=4.0*(1.0-x);
            x=1.0/(1.0+Math.exp(-x));
            x=(x-0.5)*2.0;
            x=1.0-x;
        }
        else {
            x=4.0*(-1.0-x);
            x=1.0/(1.0+Math.exp(-x));
            x=(x-0.5)*2.0;
            x=-1.0-x;
        }
        return x;
    }
    public void runOpMode() throws InterruptedException {

        Robot19888 drive = new Robot19888(hardwareMap);
        drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        boolean clawOpen = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean clawClose = gamepad1.left_bumper || gamepad2.left_bumper;





        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double denominator, frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        waitForStart();


        if (isStopRequested()) return;
        int temp = 0;
        while (opModeIsActive()) {

            low = gamepad2.x;
            mid = gamepad2.y;
            high = gamepad2.b;
            zero = gamepad2.a;
            tare = gamepad2.dpad_left;

            liftUp_M = gamepad1.y || gamepad2.b;
            liftDw_M = gamepad1.a || gamepad2.a;

            claw_open = gamepad1.x || gamepad2.left_bumper;
            claw_close = gamepad1.b || gamepad2.right_bumper;

            //y = Math.pow(gamepad1.left_stick_y,2.71828)*(gamepad1.left_stick_y/Math.abs(gamepad1.left_stick_y)); // Remember, this is reversed!
            //x = -Math.pow(gamepad1.right_stick_x,2.71828)*1.1*(gamepad1.right_stick_x/Math.abs(gamepad1.right_stick_x)); // Counteract imperfect strafing
            //rx = -Math.pow(gamepad1.left_stick_x,2.71828)*(gamepad1.left_stick_x/Math.abs(gamepad1.left_stick_x));
            y = -gamepad1.left_stick_y;
            x = gamepad1.right_stick_x * 1.1;
            rx = gamepad1.left_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftPower=ret(frontLeftPower);
            backLeftPower=ret(backLeftPower);
            frontRightPower=ret(frontRightPower);
            backRightPower=ret(backRightPower);
            drive.leftFront.setPower(frontLeftPower * limiter());
            drive.leftRear.setPower(backLeftPower * limiter());
            drive.rightFront.setPower(frontRightPower * limiter());
            drive.rightRear.setPower(backRightPower* limiter());

//            telemetry.addData("Y: ", y);
//            telemetry.addData("Y Source", gamepad1.left_stick_y);
//            telemetry.addData("X: ", x);
//            telemetry.addData("X Source", gamepad1.right_stick_x);
//            telemetry.addData("RX: ", rx);
//            telemetry.addData("RX Source", gamepad1.left_stick_x);
//            telemetry.addData("RT ", gamepad1.right_trigger);

            if (claw_open){
                drive.claw(false);
            }
            else if (claw_close){
                drive.claw(true);
            }


            //Calibration Function for Lift
            if (tare){
                drive.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //Automatic Lift Functions, untuned
            if (low){
                drive.liftOp(1900);
            }
            else if (mid){
                drive.liftOp(2400);
            }
            else if (high){
                drive.liftOp(3000);
            }
            else if (zero){
                drive.liftOp(0);
            }
            else{
                drive.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

//
//
//
//
            //Manual Lift Controls
            if (liftUp_M){
                drive.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.lift.setPower(0.7);
            }
            else if (liftDw_M){
                drive.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.lift.setPower(-0.7);
            }
            else{
                drive.lift.setPower(0);
            }

            //Claw Functions



            //Encoder Updates
//            telemetry.addData("drive.lift Encoder: ", drive.lift.getCurrentPosition());
//            telemetry.addData("Left Dist: ", drive.leftDistance.getDistance(DistanceUnit.CM));
//            telemetry.addData("Right Dist: ", drive.rightDistance.getDistance(DistanceUnit.CM));
//            telemetry.update();


        }

    }
    public double limiter() {
        if (gamepad1.left_bumper) {
            return 1.0;
        } else return 0.7;
    }

}