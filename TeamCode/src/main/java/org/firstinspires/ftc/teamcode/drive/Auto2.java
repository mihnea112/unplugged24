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


@Autonomous(name="Au_Ro_Apr 50 Done")
public class Auto2 extends LinearOpMode {
    public int team=0,sup=0,sub=0;
    private TeamElementSubsystem teamElementDetection = null;
    private AprilTagsReader aprilTagsReader =null;
    private Servo cutie;
    private DcMotorEx bratst;
    private DcMotorEx bratdr;

    private DcMotorEx intake;
    private Servo usa;
    public Pose2d end;
    public double poss=0;
    public int pos;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        Pose2d startPose = new Pose2d(14, -57, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        cutie=hardwareMap.get(Servo.class,"cutie");
        usa=hardwareMap.get(Servo.class,"usa");
        bratst = hardwareMap.get(DcMotorEx.class, "brat_st");
        bratdr = hardwareMap.get(DcMotorEx.class, "brat_dr");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        bratst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bratdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            usa.setPosition(0.3);
            teamElementDetection.setAlliance("red");
            int element_zone = teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            sleep(200);
            element_zone= teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad2", element_zone);
            dashboardTelemetry.update();
            TrajectorySequence trajSeqCaz3 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(23, -35, Math.toRadians(90)))
                    .back(8)
                    .build();
            TrajectorySequence trajSeqCaz2 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(15, -26, Math.toRadians(90)))
                    .back(8)
                    .build();
            TrajectorySequence trajSeqCaz1 = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(5, -31, Math.toRadians(140)))
                    .back(5)
                    .build();
            if(element_zone==1)
            {
                drive.followTrajectorySequence(trajSeqCaz1);
                end=trajSeqCaz1.end();
                sup=6;
                telemetry.addData("Caz1, Sup", sup);
            }
            else if(element_zone==2)
            {
                drive.followTrajectorySequence(trajSeqCaz2);
                end=trajSeqCaz2.end();
                sup=0;
                telemetry.addData("Caz2, Sup", sup);
            }
            else{
                drive.followTrajectorySequence(trajSeqCaz3);
                end=trajSeqCaz3.end();
                sup=-8;
                telemetry.addData("Caz3, Sup", sup);
            }
            teamElementDetection.stopCamera();
            cutie.setPosition(0.55);
            TrajectorySequence trajSeqP = drive.trajectorySequenceBuilder(end)
                    .addTemporalMarker(1.5, () -> {
                        bratst.setTargetPosition(2000);
                        bratst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        bratst.setPower(1);
                    })
                    .addTemporalMarker(1.5,()->{
                        bratdr.setTargetPosition(-2000);
                        bratdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        bratdr.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(44, -30+sup, Math.toRadians(0)))
                    .waitSeconds(1)
                    .build();
            TrajectorySequence trajSeqParc = drive.trajectorySequenceBuilder(trajSeqP.end())
                    .addTemporalMarker(0.8,()->{
                        cutie.setPosition(0.85);
                    })
                    .addTemporalMarker(1.7,()->{
                        usa.setPosition(0);
                    })
                    .waitSeconds(1)
                    .back(2)
                    .waitSeconds(2)
                    .addTemporalMarker(2,()->{
                        cutie.setPosition(0.59);
                        pos=0;
                        bratst.setTargetPosition(pos);
                        bratst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        bratst.setPower(1);
                        bratdr.setTargetPosition(-pos);
                        bratdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        bratdr.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(45, -57, Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(trajSeqP);
            drive.followTrajectorySequence(trajSeqParc);
            requestOpModeStop();
        }
    }
}