/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Linear OpMode", group="Linear OpMode")
public class TestAuto extends LinearOpMode {

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
    public void runOpMode() {
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

        telemetry.addData("Press Start When Ready", "");
        telemetry.update();
        waitForStart();
        sleep(15000);

        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(.5);
        sleep(150);

        frontLeft.setPower(-.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(-.5);
        sleep(6000);

        frontLeft.setPower(-.5);
        frontRight.setPower(-.5);
        backLeft.setPower(-.5);
        backRight.setPower(-.5);

        sleep(300);

        frontLeft.setPower(.5);
        frontRight.setPower(-.5);
        backLeft.setPower(-.5);
        backRight.setPower(.5);

    }
}
