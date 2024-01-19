package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleOp")

public class TeleOpeu extends LinearOpMode {
    private Servo s1;
    private Servo s2;
    private DcMotorEx brat;
    private Rev2mDistanceSensor dist_dr;
    private Rev2mDistanceSensor dist_st;
    public double poss=0;
    public int pos=0;
    public double pos2=0;
    boolean slow=false;
    public int sens = 1;
    public int sens2 = 1;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        s1=hardwareMap.get(Servo.class,"s1");
        s2=hardwareMap.get(Servo.class,"s2");
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        dist_dr = hardwareMap.get(Rev2mDistanceSensor.class,"dist_dr");
        dist_st = hardwareMap.get(Rev2mDistanceSensor.class,"dist_st");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            brat.setTargetPosition(pos);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brat.setPower(1);
            if (slow==false) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y / 3,
                                -gamepad1.left_stick_x / 3,
                                -gamepad1.right_stick_x / 3
                        )
                );
            }
            if(gamepad1.left_bumper)
            {
                slow=true;
            }
            if(gamepad1.right_bumper)
            {
                slow=false;
            }
            if(gamepad1.dpad_down && pos>0)
            {
                pos-=50;
            }
            if(gamepad1.dpad_up && 3100>pos)
            {
                pos+=50;
            }
            if(gamepad1.dpad_left)
            {
                pos2-=0.02;
            }
            if(gamepad1.dpad_right)
            {
                pos2+=0.02;

            }
            if(gamepad1.x)
            {
                s2.setPosition(0.46);
            }
            if(gamepad1.y)
            {
                s2.setPosition(0.78);
            }
            if(gamepad1.a)
            {
                s1.setPosition(0.30);
            }
            if(gamepad1.b)
            {
                s1.setPosition(0.38);
            }
            if (gamepad1.back)
            {
                sens *= -1;
                sleep(100);
            }
            if (gamepad1.start)
            {
                sens2 *= -1;
                sleep(100);
            }
            telemetry.addData("pos",pos);
            telemetry.addData("pos2",pos2);
            telemetry.addData("poss",poss);
            telemetry.addData("sens",sens);
            telemetry.addData("sens2",sens2);
            telemetry.addData("dist_dr",dist_dr.getDistance(DistanceUnit.CM));
            telemetry.addData("dist_st",dist_st.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}