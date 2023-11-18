package org.mooncat.xftc.components.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorController implements XMotorController {

    DcMotor motor;

    public MotorController(@NonNull HardwareMap hardwareMap, String motorPath) {
        motor = hardwareMap.get(DcMotor.class, motorPath);
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public void stop() {
        motor.setPower(0.0);
    }

    @Override
    public void setPosition(int position, double power) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.setTargetPosition(position);
    }

    @Override
    public int getPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void allowFreeMovement() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void resetEncoder() {
        DcMotor.RunMode currentMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(currentMode);
    }

    @Override
    public void setStallBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }
}
