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


@Autonomous(name="Au1_Alb_Apr")
public class Auto1 extends LinearOpMode {
    public int team=0;
    private Servo s1;
    private Servo intake_stanga;
    private Servo intake_dreapta;
    private DcMotorEx brat;
    private DcMotorEx intake;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 66, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        s1=hardwareMap.get(Servo.class,"cutie");
        intake_stanga=hardwareMap.get(Servo.class,"intake_st");
        intake_dreapta=hardwareMap.get(Servo.class,"intake_dr");
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(100);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
        s1.setPosition(0.6);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(33, 25, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
                    .build();
            TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                    .lineToSplineHeading(new Pose2d(12, 72, Math.toRadians(0)))
                    .back(45)
                    .build();
            TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                    .lineToSplineHeading(new Pose2d(-58, 36, Math.toRadians(0)))
                    .strafeRight(24)
                    .build();
            TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                    .forward(80)
                    .lineToSplineHeading(new Pose2d(50,30,Math.toRadians(0)))
                    .build();
        drive.followTrajectorySequence(trajSeq);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
        drive.followTrajectorySequence(trajSeq2);
        sleep(1000);
        drive.followTrajectorySequence(trajSeq3);
        sleep(1000);
        drive.followTrajectorySequence(trajSeq4);
        requestOpModeStop();
        }
    }
}