package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Controller;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "mecanum FOD Field Oriented Drive")
public class MecanumDrive extends LinearOpMode {

    public void runOpMode(){
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor SlideRight = hardwareMap.dcMotor.get("SlideRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setDirection(DcMotor.Direction.REVERSE);



        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        Controller controller = new Controller(gamepad1);

        double driveTurn;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;
        double trim = 0.0 ;
        int cycle = 0;


        waitForStart();
        while (opModeIsActive()){


            while (opModeIsActive()) {
                cycle++;
                controller.update();
                if (controller.rightBumperOnce()) trim++;
                if (controller.leftBumperOnce()) trim--;

                driveTurn = -gamepad1.right_stick_x;
                // driveVertical = -gamepad1.right_stick_y;
                // driveHorizontal = gamepad1.right_stick_x;

                gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
                gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
                gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                //finds just how much power to give the robot based on how much x and y given by gamepad
                //range.clip helps us keep our power within positive 1
                // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
                gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate));
                //the inverse tangent of opposite/adjacent gives us our gamepad degree
                robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + trim;
                if(controller.AOnce()){
                    SlideRight.setPower(0.7);
                    SlideRight.setTargetPosition(300);
                }
                //  Make robot square to the field - turn to nearest 90 degrees    RMK 20SEP2022
                if (gamepad1.right_stick_x == 0 &&
                        (((robotDegree < -0.5) && (robotDegree >= -44.5)) ||
                                ((robotDegree < -90.5) && (robotDegree >= -135.5)) ||
                                ((robotDegree < 179.5) && (robotDegree >= 135.5)) ||
                                ((robotDegree < 89.5) && (robotDegree >= 45.5))
                        )) {
                    driveTurn = driveTurn + .1;
                }

                if (gamepad1.right_stick_x == 0 &&
                        (((robotDegree < -45.5) && (robotDegree >= -89.5)) ||
                                ((robotDegree < -136.5) && (robotDegree >= -179.5)) ||
                                ((robotDegree < 134.5) && (robotDegree >= 90.5)) ||
                                ((robotDegree < 44.5) && (robotDegree >= 0.5))
                        )) {
                    driveTurn = driveTurn - .1;
                }


                // robotDegree = getAngle();
                //gives us the angle our robot is at
                movementDegree = gamepadDegree - robotDegree;
                //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
                gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the adjacent side, we can get our needed x value to power our motors
                gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the opposite side, we can get our needed y value to power our motors


                //  * again, make sure you've changed the motor names and variables to fit your team
                //   */

                //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
                //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
                //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
                frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
                backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);




            /*
             double px = gamepad1.left_stick_x;
             double py = -gamepad1.left_stick_y;
             double pa = -gamepad1.right_stick_x;

             if (Math.abs(pa) < 0.05) pa = 0;
             double p1 = -px + py - pa;
             double p2 = px + py  - pa;
             double p3 = -px + py + pa;
             double p4 = px + py + pa;
             double max = Math.max(1.0, Math.abs(p1));
             max = Math.max(max, Math.abs(p2));
             max = Math.max(max, Math.abs(p3));
             max = Math.max(max, Math.abs(p4));
             p1 /= max;
             p2 /= max;
             p3 /= max;
             p4 /= max;
             backLeft.setPower(p1);
             frontLeft.setPower(p2);
             frontRight.setPower(p3);
             backRight.setPower(p4);
             */
                //  if ( controller.rightBumperOnce() ) {
                if (cycle % 100 == 0)
                {
                    telemetry.addData("cycle", cycle);
                    telemetry.addData("trim", " %.1f", trim);
                    //      telemetry.update();
                    //     sleep(1000);
                    //   }
                    Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                    telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

                    telemetry.addData("Encoders", " %d %d %d %d", backLeft.getCurrentPosition(), frontLeft.getCurrentPosition(),
                            frontRight.getCurrentPosition(), backRight.getCurrentPosition());

                    telemetry.addData("front", "FL %.1f  FR %.1f  ", frontLeft.getPower(), frontRight.getPower());
                    telemetry.addData("back", "BL %.1f  BR %.1f  ", backLeft.getPower(), backRight.getPower());
                    telemetry.addData("slide pos",SlideRight.getCurrentPosition());
                    telemetry.update();
                }
            }
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
    }
}