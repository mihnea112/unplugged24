package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeamElementSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "VisionTest")
public class OpenCvTest extends LinearOpMode {
    private TeamElementSubsystem teamElementDetection = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        while (!opModeIsActive() && !isStopRequested()) {
            teamElementDetection.setAlliance("blue");
            int element_zone = teamElementDetection.elementDetection(telemetry);
            dashboardTelemetry.addData("Vad", element_zone);
            dashboardTelemetry.update();
            telemetry.addData("Vad", element_zone);
            telemetry.update();
            sleep(200);
        }
    }

}