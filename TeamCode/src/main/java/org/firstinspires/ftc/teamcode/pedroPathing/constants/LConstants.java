package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .00302123498232;
        ThreeWheelConstants.strafeTicksToInches = .002933545419;
        ThreeWheelConstants.turnTicksToInches = 0.003;
        ThreeWheelConstants.leftY = 6.25;
        ThreeWheelConstants.rightY = -6.25;
        ThreeWheelConstants.strafeX = -6.125;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "vl2";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "vr";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "vr2";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




