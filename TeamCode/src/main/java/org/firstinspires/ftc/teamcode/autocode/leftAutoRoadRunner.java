package org.firstinspires.ftc.teamcode.autocode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Disabled
@Autonomous(name = "LEFT", group="AutoCode")
public class leftAutoRoadRunner extends LinearOpMode {
    public class Vertical {
        private DcMotor viperLeft;
        private DcMotor viperLeft2;
        private DcMotor viperRight;
        private DcMotor viperRight2;


        public Vertical(HardwareMap hardwareMap) {
            viperLeft = hardwareMap.get(DcMotor.class, "vl");
            viperRight = hardwareMap.get(DcMotor.class, "vr");
            viperLeft2 = hardwareMap.get(DcMotor.class, "vl2");
            viperRight2 = hardwareMap.get(DcMotor.class, "vr2");


            viperRight.setDirection(DcMotorSimple.Direction.REVERSE);
            viperRight2.setDirection(DcMotorSimple.Direction.REVERSE);

            viperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }

        public class ViperUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(1);
                    viperLeft2.setPower(1);
                    viperRight.setPower(1);
                    viperRight2.setPower(1);
                    initialized = true;
                }
                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3650) {
                    return true;
                } else {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    return false;
                }
            }
        }

        public Action ViperU() {
            return new ViperUp();
        }

        public class ViperDown implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(-1);
                    viperLeft2.setPower(-1);
                    viperRight.setPower(-1);
                    viperRight2.setPower(-1);
                    initialized = true;
                }

                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1000) {
                    return true;
                } else {
                    viperLeft.setPower(0);
                    viperLeft2.setPower(0);
                    viperRight.setPower(0);
                    viperRight2.setPower(0);
                    return false;
                }

            }
        }

        public Action ViperD() {
            return new ViperDown();

        }


        public class ViperPickUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(-1);
                    viperLeft2.setPower(-1);
                    viperRight.setPower(-1);
                    viperRight2.setPower(-1);
                    initialized = true;
                }

                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1) {
                    return true;
                } else {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    return false;
                }
            }
        }

        public Action ViperPickup() {
            return new ViperPickUp();

        }

        public class ViperDrive implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(1);
                    viperLeft2.setPower(1);
                    viperRight.setPower(1);
                    viperRight2.setPower(1);
                    initialized = true;
                }

                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 400) {
                    return true;
                } else {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    return false;
                }
            }
        }

        public Action drive() {
            return new ViperDrive();

        }

        public class specimenHang implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(-1);
                    viperLeft2.setPower(-1);
                    viperRight.setPower(-1);
                    viperRight2.setPower(-1);
                    initialized = true;
                }

                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 2603) {
                    return true;
                } else {
                    viperLeft.setPower(0);
                    viperLeft2.setPower(0);
                    viperRight.setPower(0);
                    viperRight2.setPower(0);
                    return false;
                }
            }
        }

        public Action hang() {return new specimenHang();

        }


        public class specimenUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperLeft.setPower(1);
                    viperLeft2.setPower(1);
                    viperRight.setPower(1);
                    viperRight2.setPower(1);
                    initialized = true;
                }

                double pos = viperLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3165) {
                    return true;
                } else {
                    viperLeft.setPower(0);
                    viperLeft2.setPower(0);
                    viperRight.setPower(0);
                    viperRight2.setPower(0);
                    return false;
                }
            }
        }

        public Action SUp() {return new specimenUp();

        }
    }
    public class Spinner {
        private CRServo spinners;

        public Spinner(HardwareMap hardwareMap) {
            spinners = hardwareMap.get(CRServo.class, "s");
            spinners.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spinners.setPower(1);
                sleep(800);
                spinners.setPower(0);
                return false;
            }
        }


        public Action i() {
            return new intake();
        }

        public class release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spinners.setPower(-1);
                sleep(800);
                spinners.setPower(0);

                return false;
            }
        }

        public Action r() {
            return new release();
        }
    }

    public class Horizontal {
        private CRServo servoHorizontal;

        public Horizontal(HardwareMap hardwareMap) {
            servoHorizontal = hardwareMap.get(CRServo.class, "sh");
        }

        public class HOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoHorizontal.setPower(-1);
                sleep(650);
                servoHorizontal.setPower(0);




                return false;
            }
        }

        public Action OUT() { return new HOut();
        }

        public class HOut2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoHorizontal.setPower(-1);
                sleep(500);
                servoHorizontal.setPower(0);




                return false;
            }
        }

        public Action OUT2() { return new HOut2();
        }


        public class HIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoHorizontal.setPower(1);
                sleep(750);
                servoHorizontal.setPower(0);
                return false;
            }
        }

        public Action IN() { return new HIn();
        }
    }

    @Override
    public void runOpMode() {



        Pose2d beginPose = new Pose2d(0, -3, 0);
        Pose2d firstPose = new Pose2d(8, 0, 0);
        Pose2d towerPoseF = new Pose2d(10,-8,45);
        Pose2d block1 = new Pose2d(16,-28,-90);
        Pose2d block2 = new Pose2d(10,-36,-90);
//        Pose2d block3 = new Pose2d(36.5,28,0);
//        Pose2d specimenPose = new Pose2d(28, 0, 0);
        Pose2d endPose = new Pose2d(1.5,112.5,270);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ElapsedTime runtime = new ElapsedTime();
        Vertical vertical = new Vertical(hardwareMap);
        Spinner spinner = new Spinner(hardwareMap);
        Horizontal horizontal = new Horizontal(hardwareMap);

        TrajectoryActionBuilder start = drive.actionBuilder(beginPose)
                .setTangent(0)
                .splineTo(new Vector2d(14.5, -3), Math.toRadians(0))
//                .splineTo(new Vector2d(28, 0), Math.toRadians(0))
                .waitSeconds(0.01);

        TrajectoryActionBuilder one = drive.actionBuilder(firstPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11,-33,Math.toRadians(-100)),Math.toRadians(0))
                .waitSeconds(0.01);

        TrajectoryActionBuilder Return = drive.actionBuilder(block1)
//                .splineToLinearHeading(new Pose2d(10,-20,Math.toRadians(0)),Math.toRadians(90))
                .setTangent(0)
                .turn(Math.toRadians(-210))
                .strafeTo(new Vector2d(20,-10))
                .waitSeconds(0.01);


        TrajectoryActionBuilder two = drive.actionBuilder(towerPoseF)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(0,-27,Math.toRadians(-100)),Math.toRadians(0))
//                .waitSeconds(5)
//                .splineToLinearHeading(new Pose2d(0,-35,Math.toRadians(-100)),Math.toRadians(0))
                .turn(Math.toRadians(-180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0,-36,Math.toRadians(-110)),Math.toRadians(0))
                .strafeTo(new Vector2d(-7,-36))
                .waitSeconds(0.01);

        TrajectoryActionBuilder Return2 = drive.actionBuilder(block1)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20,-10,Math.toRadians(-10)),Math.toRadians(0))

//                .strafeTo(new Vector2d(19,-15))
//                .turn(Math.toRadians(135))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(0,-10,Math.toRadians(90)),Math.toRadians(0))
//                .lineToXLinearHeading(0,Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(10,-5,Math.toRadians(135)),Math.toRadians(0))
                .waitSeconds(0.1);


        Actions.runBlocking(spinner.i());

        while (!isStopRequested() && !opModeIsActive()) {


            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(


                            vertical.drive(),
                            start.build(),

                            vertical.ViperU(),
                            horizontal.OUT(),
                            spinner.r(),
                            horizontal.IN(),
                            vertical.ViperD(),

                            one.build(),
                            vertical.ViperPickup(),
                            spinner.i(),
                            vertical.drive(),
                            Return.build(),

                            vertical.ViperU(),
                            horizontal.OUT2(),
                            spinner.r(),
                            horizontal.IN(),
                            vertical.ViperD(),

                            two.build()
                    )

//                            two.build(),
//                            vertical.ViperPickup(),
//                            spinner.i(),
//                            vertical.drive(),
//                            Return2.build(),
//
//
//                            vertical.ViperU(),
//                            horizontal.OUT2(),
//                            spinner.r(),
//                            horizontal.IN(),
//                            vertical.ViperD()
            );

        }
    }
}

