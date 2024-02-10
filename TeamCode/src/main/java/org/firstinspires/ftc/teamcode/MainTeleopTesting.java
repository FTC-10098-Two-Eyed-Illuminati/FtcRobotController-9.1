package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Config
@TeleOp
public class MainTeleopTesting extends OpMode {

    private PIDController controller;
    private PIDController controllerdos;


    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.31;

    public static double p2 = 0.01, i2 = 0, d2 = 0.0001;
    public static double f2 = 0.3;

    public static int target = 0;
    public static int target2 = 0;


    private final double ticks_in_degree = 145.1/180;

    private DcMotorEx slide_motor;
    private DcMotorEx rotation_motor;


    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private Controller controller1;

    private Servo rotate;
    private Servo rotateClaw;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo launcher;
    private Controller controller2;

    public static double trim = 0.0 ;
    public static int cycle = 0;
    public int index = 0;
    public final double closed_pos = 0.4;
    public final double open_pos = 0.2;


    private SampleMecanumDrive drive;

    private ElapsedTime timer;
    private ElapsedTime timer2;

    @Override
    public void init(){
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        slide_motor = hardwareMap.get(DcMotorEx.class, "slide_motor");
        rotation_motor = hardwareMap.get(DcMotorEx.class, "rotation");
        rotate = hardwareMap.get(Servo.class, "rotate");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        launcher = hardwareMap.get(Servo.class, "launcher");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        controller = new PIDController(p, i, d);
        controllerdos = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();
        timer2 = new ElapsedTime();
        target = -10;
        target2 = 50;
        rotation_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateClaw.setPosition(0.3);
        rotate.setPosition(0.9);
        launcher.setPosition(0);


    }


    @Override
    public void loop(){

        launcher.setPosition(0+gamepad2.left_stick_x);


        cycle++;
        controller1.update();
        controller2.update();
        if (controller1.rightBumperOnce()) trim++;
        if (controller1.leftBumperOnce()) trim--;

        if (gamepad1.right_trigger == 1.0){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/3,
                            -gamepad1.left_stick_x/3,
                            -gamepad1.right_stick_x/3
                    ));
        }else{
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));

        }
        //state machine
        if (controller2.AOnce()){
            //safe open
            rotate.setPosition(0.13);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.4);
            leftClaw.setPosition(0.1);
            target = -10;
            target2 = 150;
        }
        if(controller2.leftBumperOnce()){
            //pickup position
            rotate.setPosition(0.13);
            rotateClaw.setPosition(0.33);
            rightClaw.setPosition(0.4);
            leftClaw.setPosition(0.2);
            target = -10;
            target2 = 150;
        }
        if(controller2.rightBumperOnce()){
            //retract
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            leftClaw.setPosition(0.4);
            target = -10;
            target2 = 150;
            rotateClaw.setPosition(0.3);
        }
        if(controller2.dpadUpOnce()){
            //middle score
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            rightClaw.setPosition(0.4);
            target = -50;
            target2 = 700;
        }
        if(controller2.dpadRight()){
            //score
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            rightClaw.setPosition(0.4);
            target = -300;
            target2 = 1100;
            timer.reset();
        }
        if(controller2.dpadDownOnce()){
            //middle retract
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            rightClaw.setPosition(0.4);
            target = -50;
            target2 = 1100;
        }
        if(controller2.dpadLeftOnce()){
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            rightClaw.setPosition(0.4);
            target = -50;
            target2 = 700;
        }
        if(gamepad2.back){
            rotate.setPosition(0.7);
            rotateClaw.setPosition(0.6);
            rightClaw.setPosition(0.2);
            leftClaw.setPosition(0.4);
            target = -50;
            target2 = 150;
            timer.reset();

        }
        if(controller1.BOnce()){
            launcher.setPosition(-.5);
        }
        if(timer2.milliseconds()>500){
            UpdatePID(rotation_motor,p2,i2,d2,f2,target2, controllerdos);
        }
        if(timer.milliseconds()>500){
            UpdatePID(slide_motor,p,i,d,f,target, controller);
        }



        if(controller1.XOnce()){
            leftClaw.setPosition(0.1);
            rightClaw.setPosition(0.4);
        }
        if(controller1.YOnce()){
            leftClaw.setPosition(0.3);
            rightClaw.setPosition(0.2);
        }



        telemetry.addData("cycle", cycle);
        telemetry.addData("trim", " %.1f", trim);
        telemetry.addData("Encoders", " %d %d %d %d", backLeft.getCurrentPosition(), frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(), backRight.getCurrentPosition());

        telemetry.addData("front", "FL %.1f  FR %.1f  ", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("back", "BL %.1f  BR %.1f  ", backLeft.getPower(), backRight.getPower());
        telemetry.addData("pos", slide_motor.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("launch", launcher.getPosition());
        telemetry.addData("servos data", rotate.getPosition());
        telemetry.addData("more stuff", rotateClaw.getPosition());
        telemetry.addData("even more stuff", index);
        telemetry.update();
    }


    public void UpdatePID(DcMotorEx motor, double p, double i, double d, double f, int target, PIDController controller) {
        controller.setPID(p,i,d);
        int motorpos = motor.getCurrentPosition();
        double pid = controller.calculate(motorpos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;

        motor.setPower(power);

        telemetry.addData("pos", motorpos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("actual power:", motor.getPower());
    }


}
