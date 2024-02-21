package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeamElementSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "StateTest")
public class AutoTest extends LinearOpMode {
    private TeamElementSubsystem teamElementDetection = null;

    @Override
    public void runOpMode() throws InterruptedException {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        while (!opModeIsActive() && !isStopRequested()) {
            teamElementDetection.setAlliance("blue");
            int element_zone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("Vad",element_zone);
            telemetry.update();
            sleep(1000);
        }
    }

}