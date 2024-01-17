package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
//                                .strafeLeft(20)
//                                .forward(30)
//                                .lineToSplineHeading(new Pose2d(50, 37, Math.toRadians(0)))
                                //stanga backdrop
//                                drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270)))
//                                        .strafeRight(20)
//                                        .lineToSplineHeading(new Pose2d(0, 58, Math.toRadians(270)))
//                                        .lineToSplineHeading(new Pose2d(50, 37, Math.toRadians(0)))
                                        //stanga loading
//                                drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
//                                .strafeLeft(25)
//                                .lineToSplineHeading(new Pose2d(5, -58, Math.toRadians(90)))
//                                .lineToSplineHeading(new Pose2d(50, -37, Math.toRadians(0)))
                        //      drapta loading
                                drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                        .strafeRight(20)
                                        .forward(20)
                                        .lineToSplineHeading(new Pose2d(50, -37, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}