package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Robot19888;
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
public class AutoWithAPTAG extends LinearOpMode {
    public static double DISTANCE = 60; // in
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

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

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

        telemetry.setMsTransmissionInterval(50);


        AprilTagDetection tagOfInterest = null;
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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


        Robot19888 drive = new Robot19888(hardwareMap);


        if(!isStopRequested() && opModeIsActive()) {
            Pose2d startPose = new Pose2d(72, -36, Math.toRadians(180));
            drive.setPoseEstimate(startPose);



            //Below is the Trajectory initialization for the drive code.
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .forward(6)
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .strafeRight(14)
                    .build();
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .forward(57)
                    .build();

            //drives to place location & has lift going up
            Trajectory dropPosition = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(90))),false)
                    .forward(6)
                    .build();
            //places here lift open
            Trajectory backfromDrop = drive.trajectoryBuilder(dropPosition.end())
                    .back(6)
                    .build();
            //lift down
            Trajectory strafeLeft = drive.trajectoryBuilder(backfromDrop.end())
                    .strafeLeft(8)
                    .build();
            Trajectory toJunct = drive.trajectoryBuilder(strafeLeft.end())
                    .forward(51)//make sure it doesnt ram into the thing
                    .build();
            Trajectory toLoad = drive.trajectoryBuilder(toJunct.end())
                    .back(49)
                    .build();
            Trajectory strafeRight = drive.trajectoryBuilder(toLoad.end())
                    .strafeRight(8)
                            .build();







            //put after traj 4
            drive.claw(true);
            Thread.sleep(2000);
            drive.liftOp(200);

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            drive.liftOp(3000); //lift going up here
            drive.followTrajectory(traj3);

            drive.turn(Math.toRadians(90));

            drive.followTrajectory(dropPosition);

            drive.liftOp(3000);
            drive.claw(false); //drops
            drive.liftOp(3000);

            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(450); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(1000);
            drive.liftOp(3000);
            Thread.sleep(500);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(3000);
            drive.followTrajectory(backfromDrop);

            //2ndcycl
            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(350); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(1000);
            drive.liftOp(3000);
            Thread.sleep(500);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(3000);
            drive.followTrajectory(backfromDrop);


            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(350); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(1000);
            drive.liftOp(3000);
            Thread.sleep(500);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(3000);
            drive.followTrajectory(backfromDrop);


            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(250); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(1000);
            drive.liftOp(3000);
            Thread.sleep(500);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(3000);
            drive.followTrajectory(backfromDrop);


            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(200); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(1000);
            drive.liftOp(3000);
            Thread.sleep(500);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(3000);
            drive.followTrajectory(backfromDrop);




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


