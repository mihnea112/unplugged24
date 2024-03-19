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
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


@Autonomous(name="Au_Alb_Apr Test")
public class Auto1 extends LinearOpMode {
    public int team=0,sup=0;
    private TeamElementSubsystem teamElementDetection = null;
    private AprilTagsReader aprilTagsReader =null;
    private Servo cutie;
    private DcMotorEx bratst;
    private DcMotorEx bratdr;

    private DcMotorEx intake;
    public Pose2d end;
    public double poss=0,sub=2;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        cutie=hardwareMap.get(Servo.class,"cutie");
        bratst = hardwareMap.get(DcMotorEx.class, "brat_st");
        bratdr = hardwareMap.get(DcMotorEx.class, "brat_dr");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        bratst.setDirection(DcMotorSimple.Direction.REVERSE);
        bratst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bratdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            TrajectorySequence trajSeqCaz1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(23, 35, Math.toRadians(270)))
                    .build();
            TrajectorySequence trajSeqCaz2 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(12, 32, Math.toRadians(270)))
                    .build();
            TrajectorySequence trajSeqCaz3 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(7, 35, Math.toRadians(220)))
                    .build();
            end=trajSeqCaz1.end();
            TrajectorySequence trajSeqP = drive.trajectorySequenceBuilder(end)
                    .back(8)
                    .lineToSplineHeading(new Pose2d(43, 35, Math.toRadians(0)))
                    .build();
            TrajectorySequence trajSeqParc = drive.trajectorySequenceBuilder(trajSeqP.end())
                    .lineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)))
                    .build();
        drive.followTrajectorySequence(trajSeqCaz1);
        drive.followTrajectorySequence(trajSeqP);
        drive.followTrajectorySequence(trajSeqParc);
        requestOpModeStop();
        }
    }
}