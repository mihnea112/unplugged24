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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "StateTest")
public class AutoTest extends LinearOpMode {
    private TeamElementSubsystem teamElementDetection = null;
    private AprilTagsReader aprilTagsReader =null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagsReader= new AprilTagsReader(hardwareMap);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested())
        {
            AprilTagPoseFtc distance=aprilTagsReader.telemetryAprilTag(telemetry,3);
                if(distance!=null)
                {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", distance.x, distance.y, distance.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", distance.pitch, distance.roll, distance.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", distance.range, distance.bearing, distance.elevation));
                    telemetry.update();
                }
                sleep(50);
        }
    }

}