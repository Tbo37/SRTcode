package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class horizontalSubsystem {

    private CRServo servoH;

    /**
     * This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method.
     */


    public horizontalSubsystem(HardwareMap hardwareMap) {
        servoH = hardwareMap.get(CRServo.class, "sh");
        servoH.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void horizontalIn() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 1) {
            servoH.setPower(1);
            if (timer.seconds() > 1) {
                servoH.setPower(0.1);
                break;
            }
        }
    }

    public void horizontalIn2() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 1.5) {
            servoH.setPower(1);
            if (timer.seconds() > 1.5) {
                servoH.setPower(0.1);
                break;
            }
        }
    }

    public void horizontalOut() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 0.5) {
            servoH.setPower(-1);
            if (timer.seconds() > 0.5) {
                servoH.setPower(0);
                timer.reset();
                break;
            }
        }
    }
    public void horizontalOutInitial() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 0.65) {
            servoH.setPower(-1);
            if (timer.seconds() > 0.65) {
                servoH.setPower(0);
                timer.reset();
                break;
            }
        }
    }
    public void horizontalOut2() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 0.5) {
            servoH.setPower(-1);
            if (timer.seconds() > 0.5) {
                servoH.setPower(0);
                timer.reset();
                break;
            }
        }
    }

    public void horizontalInForever() {
            servoH.setPower(0.3);

            }
        }

