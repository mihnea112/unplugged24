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
    private Servo drona;
    private Servo intake_stanga;
    private Servo intake_dreapta;
    private Servo pixel;
    private DcMotorEx brat;
    private Rev2mDistanceSensor dist_dr;
    private Rev2mDistanceSensor dist_st;
    public double poss=0;
    public int pos=0;
    boolean slow=false;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        s1=hardwareMap.get(Servo.class,"s1");
        intake_stanga=hardwareMap.get(Servo.class,"intake_st");
        intake_dreapta=hardwareMap.get(Servo.class,"intake_dr");
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        dist_dr = hardwareMap.get(Rev2mDistanceSensor.class,"dist_dr");
        dist_st = hardwareMap.get(Rev2mDistanceSensor.class,"dist_st");
        drona=hardwareMap.get(Servo.class,"drona");
        pixel=hardwareMap.get(Servo.class,"pixel");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drona.setPosition(0.6);

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
            }
            if(gamepad2.dpad_up && 3100>pos)
            {
                pos+=50;
            }
            if(gamepad2.a)
            {
                //positie intake inchis
                intake_dreapta.setPosition(0);
                intake_stanga.setPosition(0.5);
            }
            if(gamepad2.b)
            {
                //pozitie intake deschis
                intake_dreapta.setPosition(0.32);
                intake_stanga.setPosition(0.20);
            }
            if(gamepad2.x)
            {
                //pozitie intake pixel in cutie
                intake_stanga.setPosition(0.70);
            }
            if(gamepad2.y)
            {
                //positie sustinere stack
                intake_dreapta.setPosition(0.13);
            }
            if(gamepad2.right_trigger==1)
            {
                //pozitie cutie intake
                s1.setPosition(0.92);
            }
            if(gamepad2.right_bumper)
            {
                //pozitie cutie hold
                s1.setPosition(0.68);
            }
            if(gamepad2.left_trigger==1)
            {
                //pozitie cutie outtake
                s1.setPosition(0);
            }

            if(gamepad1.x){
                //trimite avion la sf petru
                drona.setPosition(0);
            }
            if(gamepad1.a){
                //hold airplane
                drona.setPosition(0.6);
            }

            //testing for reglaje

            if (gamepad1.y){
                pixel.setPosition(0.2);
            }

            if (gamepad1.b){
                //rest pos
                pixel.setPosition(1);
            }


            telemetry.addData("pos",pos);
            telemetry.addData("dist_dr",dist_dr.getDistance(DistanceUnit.CM));
            telemetry.addData("dist_st",dist_st.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}