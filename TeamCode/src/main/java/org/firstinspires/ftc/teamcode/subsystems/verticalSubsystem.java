package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class verticalSubsystem {

    private DcMotor viperLeft, viperRight, viperLeft2, viperRight2;


    public verticalSubsystem(HardwareMap hardwareMap) {
        viperLeft = hardwareMap.get(DcMotor.class, "vl");
        viperRight = hardwareMap.get(DcMotor.class, "vr");
        viperLeft2 = hardwareMap.get(DcMotor.class, "vl2");
        viperRight2 = hardwareMap.get(DcMotor.class, "vr2");

        viperRight.setDirection(DcMotorSimple.Direction.REVERSE);
        viperRight2.setDirection(DcMotorSimple.Direction.REVERSE);

        viperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void vUp() {
        double pos = viperLeft.getCurrentPosition();
        if (pos < 2850) {
            while (pos < 2850) {
                viperLeft.setPower(1);
                viperLeft2.setPower(1);
                viperRight.setPower(1);
                viperRight2.setPower(1);
                pos = viperLeft.getCurrentPosition();
                if (pos >= 2850) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
    public void vDown() {
        double pos = viperLeft.getCurrentPosition();
        if (pos > 500) {
            while (pos > 300) {
                viperLeft.setPower(-1);
                viperLeft2.setPower(-1);
                viperRight.setPower(-1);
                viperRight2.setPower(-1);
                pos = viperLeft.getCurrentPosition();
                if (pos <= 300) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
    public void vPickup() {
        double pos = viperLeft.getCurrentPosition();
        if (pos > 5) {
            while (pos > 5) {
                viperLeft.setPower(-1);
                viperLeft2.setPower(-1);
                viperRight.setPower(-1);
                viperRight2.setPower(-1);
                pos = viperLeft.getCurrentPosition();
                if (pos <= 5) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
    public void vDrive() {
        double pos = viperLeft.getCurrentPosition();
        if (pos < 300) {
            while (pos < 300) {
                viperLeft.setPower(1);
                viperLeft2.setPower(1);
                viperRight.setPower(1);
                viperRight2.setPower(1);
                pos = viperLeft.getCurrentPosition();
                if (pos >= 300) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
    // -----------------------------------------------------------------------------------------

    public void vhang() {
        double pos = viperLeft.getCurrentPosition();
        if (pos < 1115) {
            while (pos < 1115) {
                viperLeft.setPower(1);
                viperLeft2.setPower(1);
                viperRight.setPower(1);
                viperRight2.setPower(1);
                pos = viperLeft.getCurrentPosition();
                if (pos >= 1115) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
    public void vHangWall() {
        double pos = viperLeft.getCurrentPosition();
        if (pos < 650) {
            while (pos < 650) {
                viperLeft.setPower(1);
                viperLeft2.setPower(1);
                viperRight.setPower(1);
                viperRight2.setPower(1);
                pos = viperLeft.getCurrentPosition();
                if (pos >= 650) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }

    public void vDriveFromUp() {
        double pos = viperLeft.getCurrentPosition();
        if (pos > 0) {
            while (pos > 0) {
                viperLeft.setPower(-0.7);
                viperLeft2.setPower(-0.7);
                viperRight.setPower(-0.7);
                viperRight2.setPower(-0.7);
                pos = viperLeft.getCurrentPosition();
                if (pos <= 0) {
                    viperLeft.setPower(0.13);
                    viperLeft2.setPower(0.13);
                    viperRight.setPower(0.13);
                    viperRight2.setPower(0.13);
                    break;
                }
            }
        }
        else {
            viperLeft.setPower(0.13);
            viperLeft2.setPower(0.13);
            viperRight.setPower(0.13);
            viperRight2.setPower(0.13);
        }
    }
}