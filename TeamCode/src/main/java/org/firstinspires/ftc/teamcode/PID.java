package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PID extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = -200;

    private final double ticks_in_degree = 145.1/180;

    private DcMotorEx slide_motor;
    private DcMotorEx rotation_motor;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide_motor = hardwareMap.get(DcMotorEx.class, "slide_motor");
        rotation_motor = hardwareMap.get(DcMotorEx.class, "rotation");
    }

    @Override
    public void loop() {
        UpdatePID(rotation_motor);
    }

    public void UpdatePID(DcMotorEx motor) {
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
        telemetry.update();
    }
}
