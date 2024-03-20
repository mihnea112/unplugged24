package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp_2 Driver")

public class TeleOpeu extends LinearOpMode {

    private Servo cutie;
    private Servo usa;
    private Servo intake_servo;
    //private Servo drona;
    private DcMotorEx bratdr;
    private DcMotorEx bratst;
    private DcMotorEx intake;
    public double poss=0;
    public RevColorSensorV3 sensorColor1;
    public RevColorSensorV3 sensorColor2;
    public int pos=0, posh=0;
    boolean slow=false,intakes=false,close=false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        cutie=hardwareMap.get(Servo.class,"cutie");
        usa=hardwareMap.get(Servo.class,"usa");
        intake_servo=hardwareMap.get(Servo.class,"intake_servo");
        //drona=hardwareMap.get(Servo.class, "drona");
        bratdr = hardwareMap.get(DcMotorEx.class, "brat_dr");
        bratst = hardwareMap.get(DcMotorEx.class, "brat_st");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        sensorColor1 = hardwareMap.get(RevColorSensorV3.class, "pixel1");
        sensorColor2 = hardwareMap.get(RevColorSensorV3.class, "pixel2");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bratst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bratdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bratdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            //drona.setPosition(0);
            bratst.setTargetPosition(pos);
            bratst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bratst.setPower(1);
            bratdr.setTargetPosition(-pos);
            bratdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bratdr.setPower(1);
            // -- Setari Driving -- //

            // -- Gamepad 1 -- //

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
            if(gamepad1.right_bumper)
            {
                slow=false;
            }

            if(gamepad1.left_bumper)
            {
                slow=true;
            }

            if(gamepad1.triangle)
            {
                //drona.setPosition(1);
            }

            // -- Setari Brat(Viper) -- //

            if(gamepad1.dpad_down)
            {
                pos = -10;
                bratst.setTargetPosition(pos);
                bratst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bratst.setPower(1);
                bratdr.setTargetPosition(-pos);
                bratdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bratdr.setPower(1);
                bratst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bratst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bratdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bratdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // -- Gamepad 2 -- //

            if(gamepad2.dpad_down)
            {
                close = true;
                cutie.setPosition(0.59);
                pos = 0;
            }
            if(gamepad2.dpad_right)
            {
                pos=1000;
            }
            if(gamepad2.dpad_up)
            {
                pos=2000;
            }
            if(gamepad2.dpad_left)
            {
                pos=3000;
            }
            if(gamepad2.b)
            {
                close=!close;
                sleep(100);
            }
            // -- Pozitii Cutie -- //

            if(gamepad2.x)
            {
                //pozitie cutie outtake
                cutie.setPosition(0.80);
            }
            if(gamepad2.y)
            {
                //pozitie cutie intake
                cutie.setPosition(0.85);
            }
            if(gamepad2.a)
            {
                //pozitie cutie agatare
                cutie.setPosition(0.50);
            }
            if(close)
            {
                usa.setPosition(0);
            }
            else
            {
                usa.setPosition(0.3);
            }
            if(gamepad2.right_bumper)
            {
                intake_servo.setPosition(0.6);
            }
            if(gamepad2.left_bumper)
            {
                intake_servo.setPosition(0.75);
            }
            if(sensorColor1.red() > 999 || sensorColor1.blue() > 999 || sensorColor1.green() > 999 && close)
            {
                if (sensorColor2.red() > 999 || sensorColor2.blue() > 999 || sensorColor2.green() > 999)
                {
                    //close = false;
                    gamepad1.rumble(100);
                }
                gamepad1.rumble(50);
            }

            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger/3);
            telemetry.addData("pos",pos);
            telemetry.addData("poss",poss);
            telemetry.addData("frana",close);

            if (true)
            {
                dashboardTelemetry.addData("Red  ", sensorColor1.red());
                dashboardTelemetry.addData("Green", sensorColor1.green());
                dashboardTelemetry.addData("Blue ", sensorColor1.blue());
            } else
            {
                dashboardTelemetry.addData("Red 2", sensorColor2.red());
                dashboardTelemetry.addData("Green 2", sensorColor2.green());
                dashboardTelemetry.addData("Blue 2", sensorColor2.blue());
            }
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
}