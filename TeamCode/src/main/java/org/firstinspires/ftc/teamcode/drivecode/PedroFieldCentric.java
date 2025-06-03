package org.firstinspires.ftc.teamcode.drivecode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "14023PedroFieldCentricDrive", group = "Linear OpMode")
public class PedroFieldCentric extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

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

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


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


        viperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double speed = 1;
        double speedV = 1*(gamepad2.right_trigger);
        double slideLock = 1;

        pos = viperLeft.getCurrentPosition();
        telemetry.addData("ViperPosition", pos);
        telemetry.update();


        if (gamepad2.right_bumper) {
            speedV = 0.6;
        }
        else { speedV = 1;
        }

        if (gamepad1.right_bumper) {
            speed = 0.2;
        }
        else { speed = 1;
        }

        if (gamepad1.start) {
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
            follower.update();
            follower.startTeleopDrive();
        }
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
            servoH.setPower(0.04);
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


        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * (speed), -gamepad1.left_stick_x * (speed), -gamepad1.right_stick_x * (speed), false);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}