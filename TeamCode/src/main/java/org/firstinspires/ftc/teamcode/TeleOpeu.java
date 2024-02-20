package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp")

public class TeleOpeu extends LinearOpMode {
    private Servo s1;
    private Servo drona;
    private Servo intake_stanga;
    private Servo intake_dreapta;
    private Servo hangb;
    private DcMotorEx brat;
    private DcMotorEx intake;
    public double poss=0;
    public int pos=0;
    boolean slow=false,intakes=false;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        s1=hardwareMap.get(Servo.class,"cutie");
        hangb=hardwareMap.get(Servo.class,"hangb");
        intake_stanga=hardwareMap.get(Servo.class,"intake_st");
        intake_dreapta=hardwareMap.get(Servo.class,"intake_dr");
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);
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
            if(gamepad2.dpad_down && pos>0)
            {
                pos-=50;
                sleep(50);
            }
            if(gamepad2.dpad_up && 3100>pos)
            {
                pos+=50;
                sleep(50);
            }
            if(gamepad2.dpad_left)
            {
                poss-=0.01;
                sleep(100);
            }
            if(gamepad2.dpad_right)
            {
                poss+=0.01;
                sleep(100);
            }
            if(gamepad2.a)
            {
                poss=0.25;
                intake_stanga.setPosition(poss);
                intake_dreapta.setPosition(1-poss);
            }
            if(gamepad2.b)
            {
                poss=0.45;
                intake_stanga.setPosition(poss);
                intake_dreapta.setPosition(1-poss);
            }
            if(gamepad2.x)
            {
                hangb.setPosition(0);
            }
            if(gamepad2.y)
            {
                hangb.setPosition(1);
            }
            if(gamepad2.right_trigger==1)
            {
                //pozitie cutie intake
                s1.setPosition(0.82);
            }
            if(gamepad2.right_bumper)
            {
                //pozitie cutie hold
                s1.setPosition(0.64);
            }
            if(gamepad2.left_trigger==1)
            {
                //pozitie cutie outtake
                s1.setPosition(0);
            }
            if(gamepad2.left_bumper)
            {
                intakes=!intakes;
                intake.setPower(intakes ? 1 : 0);
                sleep(100);
            }
            telemetry.addData("pos",pos);
            telemetry.addData("poss",poss);
            telemetry.addData("intake",intakes);
            telemetry.addData("intake_power",intakes? 1 : 0);
            telemetry.update();
        }
    }
}