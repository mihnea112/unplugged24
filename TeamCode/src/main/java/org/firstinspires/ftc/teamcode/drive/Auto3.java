package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagsReader;
import org.firstinspires.ftc.teamcode.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name="Au_Ro_Dep 50 NDone")
public class Auto3 extends LinearOpMode {
    public int team=0,sup=0;
    private TeamElementSubsystem teamElementDetection = null;
    private AprilTagsReader aprilTagsReader =null;
    private Servo s1;
    private Servo intake_stanga;
    private Servo intake_dreapta;
    private DcMotorEx brat;
    private DcMotorEx intake;
    public Pose2d end;
    public double poss=0,sub=2;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
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
        poss=0.18;
        intake_stanga.setPosition(poss);
        intake_dreapta.setPosition(1-poss);
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested()){
            teamElementDetection.setAlliance("red");
            int element_zone = teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            sleep(200);
            element_zone= teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad2", element_zone);
            dashboardTelemetry.update();
            TrajectorySequence trajSeqCaz1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-33, -30, Math.toRadians(180)))
                    .build();
            TrajectorySequence trajSeqCaz2 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-42, -24, Math.toRadians(180)))
                    .build();
            TrajectorySequence trajSeqCaz3 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(-33, -30, Math.toRadians(0)))
                    .build();
            end=trajSeqCaz2.end();
            if(element_zone==1)
            {
                drive.followTrajectorySequence(trajSeqCaz1);
                end=trajSeqCaz1.end();
                sup=-9; //caz 1
                telemetry.addData("Caz1, Sup", sup);
            }
            else if(element_zone==2)
            {
                drive.followTrajectorySequence(trajSeqCaz2);
                end=trajSeqCaz2.end();
                telemetry.addData("Caz2, Sup", sup);
            }
            else{
                drive.followTrajectorySequence(trajSeqCaz3);
                end=trajSeqCaz3.end();
                sup=7; //caz 3
                telemetry.addData("Caz3, Sup", sup);
            }
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            //teamElementDetection.stopCamera();
            aprilTagsReader= new AprilTagsReader(hardwareMap);
            TrajectorySequence trajSeqP = drive.trajectorySequenceBuilder(end)
                    .addTemporalMarker(5, () -> {
                        brat.setTargetPosition(2500);
                        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        brat.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(-35, -15, Math.toRadians(0)))
                    .forward(83)
                    .lineToSplineHeading(new Pose2d(48, -35+sup, Math.toRadians(0)))
                    .addTemporalMarker(6, () -> {
                        s1.setPosition(0);
                    })
                    .waitSeconds(0.5)
                    .build();
            AprilTagPoseFtc distance=aprilTagsReader.telemetryAprilTag(telemetry, element_zone);
            if(distance!=null)
            {
                dashboardTelemetry.addData("Range", distance.range);
                dashboardTelemetry.update();
                if(distance.range<12)
                {
                    sub=4;
                }
                if(distance.range>=12)
                {
                    sub=0;
                }
            }
            else {
                dashboardTelemetry.addLine("NU Vad");
                dashboardTelemetry.update();
            }
            TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeqP.end())
                    .back(1)
                    .waitSeconds(0.5)
                    .back(2)
                    .waitSeconds(0.5)
                    .build();
            TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                    .addTemporalMarker(0.1, () -> {
                        s1.setPosition(0.64);
                        brat.setTargetPosition(0);
                        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        brat.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(50, -13, Math.toRadians(0)))
                    .forward(10)
                    .build();
            poss=0.25;
            intake_stanga.setPosition(poss);
            intake_dreapta.setPosition(1-poss);
            sleep(500);
            intake.setPower(-0.5);
            sleep(250);
            intake.setPower(0);
            sleep(4000);
            drive.followTrajectorySequence(trajSeqP);
            s1.setPosition(0);
            drive.followTrajectorySequence(trajSeq2);
            drive.followTrajectorySequence(trajSeq3);
            requestOpModeStop();
        }
    }
}