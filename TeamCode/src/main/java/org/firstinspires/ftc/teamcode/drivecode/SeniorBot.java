package org.firstinspires.ftc.teamcode.drivecode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "Old Drive Code", group = "Linear OpMode")
public class SeniorBot extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private CRServo servoH;
    private DcMotor viperLeft;
    private DcMotor viperRight;
    private DcMotor viperLeft2;
    private DcMotor viperRight2;
    private CRServo spinners;


    double pos = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        front_right = hardwareMap.get(DcMotor.class,"fr");
        front_left = hardwareMap.get(DcMotor.class,"fl");
        back_right = hardwareMap.get(DcMotor.class,"br");
        back_left = hardwareMap.get(DcMotor.class,"bl");

        servoH = hardwareMap.get(CRServo.class,"sh");

        viperLeft = hardwareMap.get(DcMotor.class,"vl");
        viperRight = hardwareMap.get(DcMotor.class,"vr");
        viperLeft2 = hardwareMap.get(DcMotor.class,"vl2");
        viperRight2 = hardwareMap.get(DcMotor.class,"vr2");

        spinners = hardwareMap.get(CRServo.class,"s");
        servoH.setDirection(DcMotorSimple.Direction.REVERSE);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        viperRight.setDirection(DcMotorSimple.Direction.REVERSE);
        viperRight2.setDirection(DcMotorSimple.Direction.REVERSE);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double speed = 1;
        double speedV = 1*(gamepad2.right_trigger);
        double slideLock = 1;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStopRequested()) {


            while (opModeIsActive()) {

                pos = viperLeft.getCurrentPosition();
                telemetry.addData("ViperPosition", pos);
                telemetry.update();

                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.start) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                if (gamepad2.right_bumper) {
                    speedV = 0.6;
                }
                else { speedV = 1;
                }

                if (gamepad1.right_bumper) {
                    speed = 0.45;
                }
                else { speed = 1;
                }


                front_left.setPower(frontLeftPower*speed);
                back_left.setPower(backLeftPower*speed);
                front_right.setPower(frontRightPower*speed);
                back_right.setPower(backRightPower*speed);


                if (gamepad2.a && gamepad2.b) {
                    slideLock = 0;
                    telemetry.addData("slidelock", "0");
                    telemetry.update();

                }
                if (slideLock == 0) {
                    viperLeft.setPower(-0.6);
                    viperRight.setPower(-0.6);
                    viperLeft2.setPower(-0.6);
                    viperRight2.setPower(-0.6);
                    telemetry.addData("Slidelock", "Initialized");
                    telemetry.update();

                }
                else if(gamepad2.dpad_up) {
                    viperLeft.setPower(1);
                    viperRight.setPower(1);
                    viperLeft2.setPower(1);
                    viperRight2.setPower(1);
                }
                else if (gamepad2.dpad_down) {
                    viperLeft.setPower(-1 * speedV);
                    viperRight.setPower(-1 * speedV);
                    viperLeft2.setPower(-1 * speedV);
                    viperRight2.setPower(-1 * speedV);
                }

                else {
                    viperLeft.setPower(0.13*slideLock);
                    viperRight.setPower(0.13*slideLock);
                    viperLeft2.setPower(0.13*slideLock);
                    viperRight2.setPower(0.13*slideLock);
                }


                if (gamepad2.y) {
                    servoH.setPower(-1);
                }
                else if (gamepad2.a) {
                    servoH.setPower(1);
                }
                else {
                    servoH.setPower(0);
                }
                if (gamepad2.left_bumper) {
                    spinners.setPower(1);
                }
                else if (gamepad2.right_bumper) {
                    spinners.setPower(-1);
                }
                else {
                    spinners.setPower(0);
                }

            }
        }
    }
}
