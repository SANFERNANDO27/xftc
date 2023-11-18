package org.mooncat.xftc.components.sensors.imu;

public interface XIMU {

    double getXAngleDeg();
    double getYAngleDeg();
    double getZAngleDeg();

    double getXAngularVelocityDeg();
    double getYAngularVelocityDeg();
    double getZAngularVelocityDeg();

    double getXAngleRad();
    double getYAngleRad();
    double getZAngleRad();

    double getXAngularVelocityRad();
    double getYAngularVelocityRad();
    double getZAngularVelocityRad();

    void resetZAngle();

}
