package org.mooncat.xftc.old.components.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface XMotorController {

    void setPower(double power);
    double getPower();
    void stop();

    void setPosition(int position, double power);
    int getPosition();

    void allowFreeMovement();
    void resetEncoder();

    void setStallBehavior(DcMotor.ZeroPowerBehavior behavior);

}
