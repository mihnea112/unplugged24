package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionEasy;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagsReader;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "StateTest")
public class AutoTest extends LinearOpMode {
    private TeamElementSubsystem teamElementDetection = null;
    private AprilTagsReader aprilTagsReader = new AprilTagsReader();

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagsReader.initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        aprilTagsReader.UpdateAprilTag();
        while(opModeIsActive() && !isStopRequested())
        {
                aprilTagsReader.UpdateAprilTag();
                sleep(50);
        }
    }

}