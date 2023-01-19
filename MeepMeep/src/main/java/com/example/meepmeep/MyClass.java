package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-30.50, -65, Math.toRadians(90));
        Pose2d placeReady = new Pose2d(-15, -9, Math.toRadians(150));
        Pose2d pickupPos = new Pose2d(-64,-12,Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52, 30, Math.toRadians(180), Math.toRadians(180), 12.53)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(placeReady)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(pickupPos,Math.toRadians(180))
                                .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}