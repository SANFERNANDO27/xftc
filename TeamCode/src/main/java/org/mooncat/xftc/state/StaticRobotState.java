package org.mooncat.xftc.state;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class StaticRobotState {
    public static final RevHubOrientationOnRobot HUB_ORIENTATION = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    );
}
