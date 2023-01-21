package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Robot19888;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class A1SplineLeft extends LinearOpMode {
    public static double startx = -26;
    public static double starty = -65;
    public static Pose2d startPose = new Pose2d(startx, starty, Math.toRadians(90));
    public static Pose2d placeReady = new Pose2d(-24, -9, Math.toRadians(90));
    public static Vector2d placeReadyV = new Vector2d(-24,-9);
    public static double pickx = -54;
    public static double picky = -10;

    public static Pose2d pickupPos = new Pose2d(pickx,picky,Math.toRadians(180));
    public static double gotosTan1 = 20;
    public static double gotosTan2 = 160;
    public static double pickupTan1 = 200;
    public static double pickupTan2 = 180;
    public static double backtoTan1 = 0;
    public static double backtoTan2 = 0;
    public static double DISTANCE = 60; // in
    public static int fullUp = 3200;
    public static int pickupOne = 550;
    public static int pickupTwo = 350;
    public static int pickupThree = 150;
    public static int pickupFour = 50;
    public static int pickupFive = 0;//recalib

    public static Pose2d parkOne = new Pose2d(-26, -65, Math.toRadians(90));
    public static Pose2d parkTwo = new Pose2d(-26, -65, Math.toRadians(90));
    public static Pose2d parkThr = new Pose2d(-26, -65, Math.toRadians(90));
    



    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int det = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.setMsTransmissionInterval(50);


        AprilTagDetection tagOfInterest = null;
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */


        Robot19888 drive = new Robot19888(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            drive.claw(true);
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if(tag.id>=0 && tag.id<=2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        det = tag.id + 1;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    //AprilTagDetectionPipeline.tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        //tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    //tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }



        if(!isStopRequested() && opModeIsActive()) {

            drive.setPoseEstimate(startPose);


            TrajectorySequence gotos = drive.trajectorySequenceBuilder(startPose)
                    .setTangent(Math.toRadians(gotosTan1))
                    .splineToConstantHeading(placeReadyV, Math.toRadians(gotosTan2))
                    .build();

            TrajectorySequence pickup = drive.trajectorySequenceBuilder(placeReady)
                    .setTangent(Math.toRadians(pickupTan1))
                    .splineToLinearHeading(pickupPos,Math.toRadians(pickupTan2))
                    .build();

            TrajectorySequence backto = drive.trajectorySequenceBuilder(pickupPos)
                    .setTangent(Math.toRadians(backtoTan1))
                    .splineToLinearHeading(placeReady,Math.toRadians(backtoTan2))
                    .build();


            //Below is the Trajectory initialization for the drive code.








            //put after traj 4
            int conestack = 5;
            drive.claw(true);
            drive.liftOp(200);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(gotos);
            drive.claw(false);
            drive.liftOp(450);
            conestack--;
            drive.followTrajectorySequence(pickup);
            drive.claw(true);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(backto);

            drive.claw(false);
            drive.liftOp(450-((5-conestack)*70));
            conestack--;
            drive.followTrajectorySequence(pickup);
            drive.claw(true);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(backto);

            drive.claw(false);
            drive.liftOp(450-((5-conestack)*70));
            conestack--;
            drive.followTrajectorySequence(pickup);
            drive.claw(true);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(backto);

            drive.claw(false);
            drive.liftOp(450-((5-conestack)*70));
            conestack--;
            drive.followTrajectorySequence(pickup);
            drive.claw(true);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(backto);

            drive.claw(false);
            drive.liftOp(450-((5-conestack)*70));
            conestack--;
            drive.followTrajectorySequence(pickup);
            drive.claw(true);
            drive.liftOp(fullUp);
            drive.followTrajectorySequence(backto);





            //drive.followTrajectory(fromPole);


            if (det==0){
                telemetry.addData("Gonna go to 1 bc I didn't see",det);

            }
            else if (det==1){
                telemetry.addData("One",det);

            }else if (det==2){
                telemetry.addData("Two",det);

            }else if (det==3){
                telemetry.addData("Three",det);

            }
            //drive.liftOp(2000);

            telemetry.addData("Lift Enc: ",drive.lift.getCurrentPosition());





            telemetry.update();


        }


    }}


