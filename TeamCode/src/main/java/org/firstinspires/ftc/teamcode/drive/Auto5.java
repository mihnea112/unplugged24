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
import org.firstinspires.ftc.teamcode.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Au_Alb_Apr 50 Done")
public class Auto5 extends LinearOpMode {
    public int team=0,sup=0;
    private TeamElementSubsystem teamElementDetection = null;
    private Servo s1;
    private Servo intake_stanga;
    private Servo intake_dreapta;
    private DcMotorEx brat;
    private DcMotorEx intake;
    public Pose2d end;
    public double poss=0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14, 64, Math.toRadians(270));
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
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            teamElementDetection.setAlliance("blue");
            int element_zone = teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            sleep(200);
            element_zone= teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad2", element_zone);
            dashboardTelemetry.update();
            TrajectorySequence trajSeqCaz1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(36, 30, Math.toRadians(0)))
                    .build();
            TrajectorySequence trajSeqCaz2 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(30, 27, Math.toRadians(0)))
                    .build();
            TrajectorySequence trajSeqCaz3 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(14, 32, Math.toRadians(0)))
                    .build();
            if(element_zone==1)
            {
                drive.followTrajectorySequence(trajSeqCaz1);
                end=trajSeqCaz1.end();
                sup=8; //caz 1
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
                sup=-8; //caz 3
                telemetry.addData("Caz3, Sup", sup);
            }
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            telemetry.update();
            teamElementDetection.stopCamera();
            TrajectorySequence trajSeqP = drive.trajectorySequenceBuilder(end)
                    .addSpatialMarker(new Vector2d(35, 40+sup), () -> {
                        brat.setTargetPosition(2700);
                        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        brat.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(45, 38+sup, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(42, 38+sup), () -> {
                        s1.setPosition(0);
                    })
                    .waitSeconds(0.5)
                    .build();
            TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeqP.end())
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
                    .lineToSplineHeading(new Pose2d(45, 58, Math.toRadians(0)))
                    .build();

            intake.setPower(-0.5);
            sleep(500);
            intake.setPower(0);
            drive.followTrajectorySequence(trajSeqP);
            s1.setPosition(0);
            drive.followTrajectorySequence(trajSeq2);
            drive.followTrajectorySequence(trajSeq3);
            requestOpModeStop();
        }
    }
}