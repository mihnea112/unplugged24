package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Au4_Alb_Dep")
public class Auto4 extends LinearOpMode {
    public Rev2mDistanceSensor dist_dr;
    public Rev2mDistanceSensor dist_st;
    public int team=0,sup=0;
    public Servo s1, pixel, drona;
    public Pose2d end;
    public DcMotorEx brat;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-36, 72, Math.toRadians(180));

        drive.setPoseEstimate(startPose);
        dist_dr = hardwareMap.get(Rev2mDistanceSensor.class,"dist_dr");
        dist_st = hardwareMap.get(Rev2mDistanceSensor.class,"dist_st");
        s1=hardwareMap.get(Servo.class, "s1");
        pixel=hardwareMap.get(Servo.class, "pixel");
        drona=hardwareMap.get(Servo.class, "drona");

        brat = hardwareMap.get(DcMotorEx.class, "brat");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ridicare brat inainte de init
        brat.setTargetPosition(100);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);

        drona.setPosition(0.6);
        s1.setPosition(0.6);
        pixel.setPosition(1);

        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            // Example spline path from SplineTest.java
            // Make sure the start pose matches with the localizer's start pose
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(5)
                    .forward(7)
                    .build();
            TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                    .forward(11)
                    .build();
            TrajectorySequence trajSeqPix1 = drive.trajectorySequenceBuilder(trajSeq2.end())
                    .lineTo(new Vector2d(-57, 27))
                    .build();
            TrajectorySequence trajSeqPix2 = drive.trajectorySequenceBuilder(trajSeq2.end())
                    .lineTo(new Vector2d(-45, 15))
                    .build();
            TrajectorySequence trajSeqPix3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                    .lineTo(new Vector2d(-34, 27))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            if(dist_st.getDistance(DistanceUnit.CM)<150)
            {
                team=2;
            }
            telemetry.addLine(String.valueOf(team));
            telemetry.update();
            sleep(200);
            drive.followTrajectorySequence(trajSeq2);
            if(dist_st.getDistance(DistanceUnit.CM)<150 && team==0)
            {
                team=1;
            }
            else if(team==0)
            {
                team=3;
            }
            telemetry.addLine(String.valueOf(team));
            telemetry.update();
            sleep(2000);
            if(team==1)
            {
                drive.followTrajectorySequence(trajSeqPix1);
                end=trajSeqPix1.end();
            }
            else if(team==2)
            {
                drive.followTrajectorySequence(trajSeqPix2);
                end=trajSeqPix2.end();
            }
            else if(team==3)
            {
                drive.followTrajectorySequence(trajSeqPix3);
                sup=-12;
                end=trajSeqPix3.end();
            }
            //aici lasi jos pixelul
            pixel.setPosition(0.2);
            sleep(3000);
            TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(end)
                    .lineTo(new Vector2d(-70, 50))
                    .build();
            drive.followTrajectorySequence(trajSeq3);

            s1.setPosition(0.6);

            brat.setTargetPosition(0);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brat.setPower(1);
            sleep(2000);
            requestOpModeStop();
        }
    }
}