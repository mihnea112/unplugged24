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
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-42, 35, Math.toRadians(220)))
                                        //caz 2.lineToSplineHeading(new Pose2d(-40, 23, Math.toRadians(0)))
                                        //caz 1.lineToSplineHeading(new Pose2d(-27, 35, Math.toRadians(320)))
                                        .back(10)
                                        .lineToSplineHeading(new Pose2d(-37, 12, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(35, 13, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(43, 35, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)))

         .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//albastru apr
//drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90)))
////caz 1.lineToSplineHeading(new Pose2d(23, 35, Math.toRadians(270)))
////caz 2.lineToSplineHeading(new Pose2d(12, 32, Math.toRadians(270)))
//                                        .lineToSplineHeading(new Pose2d(7, 35, Math.toRadians(220)))
//                                                .back(8)
//                                                .lineToSplineHeading(new Pose2d(43, 35, Math.toRadians(0)))
//                                                //parcare .lineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)))
//                                                .lineToSplineHeading(new Pose2d(35, 13, Math.toRadians(0)))
//                                                .lineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(0)))
//                                                .lineToSplineHeading(new Pose2d(35, 13, Math.toRadians(0)))
//                                                .lineToSplineHeading(new Pose2d(43, 35, Math.toRadians(0)))
//                                                .lineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)))
//
//                                                .build()
//rosu apr
// drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
//         .lineToSplineHeading(new Pose2d(23, -35, Math.toRadians(90)))
//         //caz 2.lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(90)))
//         //caz 1.lineToSplineHeading(new Pose2d(7, -35, Math.toRadians(140)))
//         .back(8)
//         .lineToSplineHeading(new Pose2d(43, -35, Math.toRadians(0)))
//         //parcare .lineToSplineHeading(new Pose2d(48, -60, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(35, -13, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(35, -13, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(43, -35, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(48, -60, Math.toRadians(0)))
//
//         .build()
//rosu dep
// drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
//         //caz 1.lineToSplineHeading(new Pose2d(-42, -35, Math.toRadians(140)))
//         //caz 2.lineToSplineHeading(new Pose2d(-40, -23, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(-27, -35, Math.toRadians(50)))
//         .back(10)
//         .lineToSplineHeading(new Pose2d(-37, -12, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(35, -13, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(43, -35, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(48, -60, Math.toRadians(0)))
//
//         .build()
//albstru dep
// drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270)))
//         .lineToSplineHeading(new Pose2d(-42, 35, Math.toRadians(220)))
//         //caz 2.lineToSplineHeading(new Pose2d(-40, 23, Math.toRadians(0)))
//         //caz 1.lineToSplineHeading(new Pose2d(-27, 35, Math.toRadians(320)))
//         .back(10)
//         .lineToSplineHeading(new Pose2d(-37, 12, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(35, 13, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(43, 35, Math.toRadians(0)))
//         .lineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)))
//
//         .build()
