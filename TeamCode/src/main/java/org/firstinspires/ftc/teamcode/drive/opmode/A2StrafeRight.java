package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Robot19888;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(group = "drive")
public class A2StrafeRight extends LinearOpMode {
    public static double strafeOne = 14;
    public static double forwardToPos = 64;
    public static double toDrop = 3;
    public static double strafetoJunctLock = 9;
    public static double toJunctDist = 49;
    public static int toTop =  3200;
    public static int toFifth=  425;
    public static int toFourth=  365;
    public static int threadSleep = 200;
    
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
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
                telemetry.addLine("Camera Error!");
                telemetry.update();

            }
        }
        );



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        AprilTagDetection tagOfInterest = null;
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
                    telemetry.addLine("Tag of interest is in sight!");
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
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


        //Below is the Trajectory initialization for the drive code.
        Pose2d startPose = new Pose2d(72, -36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //Trajectory traj1 = drive.trajectoryBuilder(startPose).forward(6).build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .strafeRight(strafeOne)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(forwardToPos)
                .build();

        //drives to place location & has lift going up
        Trajectory dropPosition = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(90))),false)
                .forward(toDrop)
                .build();
        //places here lift open
        Trajectory backfromDrop = drive.trajectoryBuilder(dropPosition.end())
                .back(toDrop)
                .build();
        //lift down
        Trajectory strafeLeft = drive.trajectoryBuilder(backfromDrop.end())
                .strafeLeft(strafetoJunctLock)
                .build();
        Trajectory toJunct = drive.trajectoryBuilder(strafeLeft.end())
                .forward(toJunctDist)
                .build();
        Trajectory toLoad = drive.trajectoryBuilder(toJunct.end())
                .back(toJunctDist)
                .build();
        Trajectory to2 = drive.trajectoryBuilder(toJunct.end())
                .back(toJunctDist-18)
                .build();
        Trajectory strafeRight = drive.trajectoryBuilder(toLoad.end())
                .strafeRight(strafetoJunctLock)
                .build();
        Trajectory to1 = drive.trajectoryBuilder(toJunct.end())
                .back(6).build();
        Trajectory to3 = drive.trajectoryBuilder(toJunct.end())
                .back(52).build();

        if(!isStopRequested() && opModeIsActive()) {
            camera = null;
            drive.liftOp(200);
            //drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            drive.liftOp(toTop); //lift going up here
            drive.followTrajectory(traj3);

            drive.turn(Math.toRadians(90));

            drive.followTrajectory(dropPosition);

            drive.liftOp(toTop);
            drive.claw(false); //drops
            drive.liftOp(toTop);

            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(toFifth); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(threadSleep);
            drive.liftOp(toTop);
            Thread.sleep(threadSleep);
            drive.followTrajectory(toLoad);
            drive.followTrajectory(strafeRight);
            drive.followTrajectory(dropPosition);
            drive.claw(false);
            drive.liftOp(toTop);
            drive.followTrajectory(backfromDrop);

            //2ndcycl
            drive.followTrajectory(backfromDrop); //backs out
            //drive.claw(false);
            drive.followTrajectory(strafeLeft);
            drive.liftOp(toFourth); //needs tuning towards the first cone
            drive.followTrajectory(toJunct);
            drive.claw(true);
            Thread.sleep(threadSleep);
            drive.liftOp(1000);
            Thread.sleep(600);
//            Thread.sleep(threadSleep);
//            drive.followTrajectory(toLoad);
//            drive.followTrajectory(strafeRight);
//            drive.followTrajectory(dropPosition);
//            drive.claw(false);
//            drive.liftOp(toTop);
//            drive.followTrajectory(backfromDrop);
//            drive.followTrajectory(strafeLeft);



//            drive.followTrajectory(backfromDrop); //backs out
//            //drive.claw(false);
//            drive.followTrajectory(strafeLeft);
//            drive.liftOp(toThird); //needs tuning towards the first cone
//            drive.followTrajectory(toJunct);
//            drive.claw(true);
//            Thread.sleep(threadSleep);
//            drive.liftOp(toTop);
//            Thread.sleep(threadSleep);

//            drive.followTrajectory(strafeRight);
//            drive.followTrajectory(dropPosition);
//            drive.claw(false);
//            drive.liftOp(toTop);
//            drive.followTrajectory(backfromDrop);


//            drive.followTrajectory(backfromDrop); //backs out
//            //drive.claw(false);
//            drive.followTrajectory(strafeLeft);
//            drive.liftOp(150); //needs tuning towards the first cone
//            drive.followTrajectory(toJunct);
//            drive.claw(true);
//            Thread.sleep(1000);
//            drive.liftOp(toTop);
//            Thread.sleep(500);
//            drive.followTrajectory(toLoad);
//            drive.followTrajectory(strafeRight);
//            drive.followTrajectory(dropPosition);
//            drive.claw(false);
//            drive.liftOp(toTop);
//            drive.followTrajectory(backfromDrop);


//            drive.followTrajectory(backfromDrop); //backs out
//            //drive.claw(false);
//            drive.followTrajectory(strafeLeft);
//            drive.liftOp(50); //needs tuning towards the first cone
//            drive.followTrajectory(toJunct);
//            drive.claw(true);
//            Thread.sleep(1000);
//            drive.liftOp(toTop);
//            Thread.sleep(500);
//            drive.followTrajectory(toLoad);
//            drive.followTrajectory(strafeRight);
//            drive.followTrajectory(dropPosition);
//            drive.claw(false);
//            drive.liftOp(toTop);
//            drive.followTrajectory(backfromDrop);




            //drive.followTrajectory(fromPole);


            if (det==0){
                telemetry.addData("Gonna go to 1 bc I didn't see",det);
                //go to 3 in place for tall
                drive.followTrajectory(to3);
                drive.liftOp(0);
            }
            else if (det==1){
                telemetry.addData("One",det);
                drive.followTrajectory(to1);
                drive.liftOp(0);

            }else if (det==2){
                telemetry.addData("Two",det);
                drive.followTrajectory(to2);
                drive.liftOp(0);



            }else if (det==3){
                drive.followTrajectory(to3);
                drive.liftOp(0);


            }
            //drive.liftOp(threadSleep0);

            telemetry.addData("Lift Enc: ",drive.lift.getCurrentPosition());

            telemetry.update();


        }


    }}


