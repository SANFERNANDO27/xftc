package org.mooncat.xftc.old.components.sensors.imu;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RevHubIMU implements XIMU {

    private final IMU imu;

    public RevHubIMU(@NonNull HardwareMap hardwareMap, ImuOrientationOnRobot imuOrientation) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(imuOrientation));
    }

    public RevHubIMU(@NonNull HardwareMap hardwareMap, IMU.Parameters parameters) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public double getXAngleDeg() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    @Override
    public double getYAngleDeg() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    @Override
    public double getZAngleDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public double getXAngularVelocityDeg() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate;
    }

    @Override
    public double getYAngularVelocityDeg() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate;
    }

    @Override
    public double getZAngularVelocityDeg() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    @Override
    public double getXAngleRad() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
    }

    @Override
    public double getYAngleRad() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
    }

    @Override
    public double getZAngleRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public double getXAngularVelocityRad() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;
    }

    @Override
    public double getYAngularVelocityRad() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
    }

    @Override
    public double getZAngularVelocityRad() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @Override
    public void resetZAngle() {
        imu.resetYaw();
    }
}
