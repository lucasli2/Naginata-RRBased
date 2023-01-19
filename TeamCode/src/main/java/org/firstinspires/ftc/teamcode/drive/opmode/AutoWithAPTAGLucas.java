package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Robot19888;
import org.openftc.easyopencv.OpenCvCamera;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoWithAPTAGLucas extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor leftdistance = hardwareMap.get(DistanceSensor .class, "leftDistance");
        DistanceSensor rightdistance = hardwareMap.get(DistanceSensor . class,"rightDistance");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        double st= leftdistance.getDistance(DistanceUnit.INCH);

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;


        Robot19888 drive = new Robot19888(hardwareMap);




//        Trajectory toJunct = drive.trajectoryBuilder(new Pose2d())
//                .forward(5)
        waitForStart();
        //detect the cone


//        if (isStopRequested()) return;
//        drive.followTrajectory(toJunct);
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

        drive.liftOp(300);
    }
}
