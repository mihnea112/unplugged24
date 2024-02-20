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
                                drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
         //pixel caz 1 albastru apr
         //.lineToSplineHeading(new Pose2d(12, 30, Math.toRadians(0)))
         //pixel caz 2 albastru apr
         //.lineToSplineHeading(new Pose2d(28, 24, Math.toRadians(0)))
         //pixel caz 3 albastru apr
         .lineToSplineHeading(new Pose2d(33, 30, Math.toRadians(0)))
         .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
                                           .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(-35, 60, Math.toRadians(0)))
         .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                                        .back(3)
                                        .lineToSplineHeading(new Pose2d(-45, 10, Math.toRadians(0)))
         .forward(80)
         .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
         .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//drive.trajectorySequenceBuilder(new Pose2d(-33, 60, Math.toRadians(270)))
//        //albastru dep
//        //pixel caz 3 albstru dep
//        //.lineToSplineHeading(new Pose2d(-35, 30, Math.toRadians(180)))
//        //pixel caz 2 albastru dep
//        //.lineToSplineHeading(new Pose2d(-42, 24, Math.toRadians(180)))
//        //pixel caz 1 albastru dep
//        .lineToSplineHeading(new Pose2d(-37, 25, Math.toRadians(0)))
//        .lineToSplineHeading(new Pose2d(-56, 10, Math.toRadians(0)))
//        .forward(80)
//        .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//        .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(0)))
//        //ciclu
//        .back(40)
//        .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
//        .strafeRight(24)
//        .forward(80)
//        .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//        .build()
//rosu apr
// drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
//         //pixel caz 3 rosu apr
//         .lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(50,-30,Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(12, -60, Math.toRadians(0)))
//         .back(40)
//         .lineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(0)))
//         .strafeLeft(24)
//         .forward(80)
//         .lineToSplineHeading(new Pose2d(50,-30,Math.toRadians(0)))
//   //pixel caz 2 rosu apr
//     //.lineToSplineHeading(new Pose2d(28, -24, Math.toRadians(0)))
//  //pixel caz 1 rosu apr
//  //.lineToSplineHeading(new Pose2d(35, -30, Math.toRadians(0)))
//  /.build()

//rosu dep
// drive.trajectorySequenceBuilder(new Pose2d(-33, -60, Math.toRadians(90)))
////                                        //pixel caz 1 rosu dep
////                                        //.lineToSplineHeading(new Pose2d(-33, -30, Math.toRadians(180)))
////                                        //pixel caz 2 rosu dep
////                                        //.lineToSplineHeading(new Pose2d(-42, -24, Math.toRadians(180)))
////                                        //pixel caz 3 rosu dep
//         .lineToSplineHeading(new Pose2d(-37, -25, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(-56, -10, Math.toRadians(0)))
//         .forward(80)
//         .lineToSplineHeading(new Pose2d(50,-30,Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(12, -60, Math.toRadians(0)))
//         //ciclu
//         .back(40)
//         .lineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(0)))
//         .strafeLeft(24)
//         .forward(80)
//         .lineToSplineHeading(new Pose2d(50,-30,Math.toRadians(0)))
//         .build()

//albastru apr
// drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
//         //pixel caz 1 albastru apr
//         //.lineToSplineHeading(new Pose2d(12, 30, Math.toRadians(0)))
//         //pixel caz 2 albastru apr
//         //.lineToSplineHeading(new Pose2d(28, 24, Math.toRadians(0)))
//         //pixel caz 3 albastru apr
//         .lineToSplineHeading(new Pose2d(33, 30, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(0)))
//         .back(40)
//         .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
//         .strafeRight(24)
//         .forward(80)
//         .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//         .build()

//albastru dep
//drive.trajectorySequenceBuilder(new Pose2d(-33, 60, Math.toRadians(270)))
//        //pixel caz 3 rosu dep
//        //.lineToSplineHeading(new Pose2d(-35, 30, Math.toRadians(180)))
//        //pixel caz 2 rosu dep
//        //.lineToSplineHeading(new Pose2d(-42, 24, Math.toRadians(180)))
//        //pixel caz 1 albastru dep
//        .lineToSplineHeading(new Pose2d(-37, 25, Math.toRadians(0)))
//        .lineToSplineHeading(new Pose2d(-56, 10, Math.toRadians(0)))
//        .forward(80)
//        .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//        .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(0)))
//        //ciclu
//        .back(40)
//        .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
//        .strafeRight(24)
//        .forward(80)
//        .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
//        .build()