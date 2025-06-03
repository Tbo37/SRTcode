package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class spinnerSubsystem {

    private CRServo spinners;


    /**
     * This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method.
     */

    public spinnerSubsystem(HardwareMap hardwareMap) {
        spinners = hardwareMap.get(CRServo.class, "s");
    }
    public void spinIn() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 1) {
            spinners.setPower(-1);
            if (timer.seconds() > 1) {
                spinners.setPower(0);
                break;
            }
        }
    }
    public void spinOut() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 0.7) {
            spinners.setPower(1);
            if (timer.seconds() > 0.7) {
                spinners.setPower(0);
                break;
            }
        }
    }
    public void spinWait() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() <= 0.2) {
            spinners.setPower(-1);
            if (timer.seconds() > 0.2) {
                spinners.setPower(0);
                break;
            }
        }
    }
}

