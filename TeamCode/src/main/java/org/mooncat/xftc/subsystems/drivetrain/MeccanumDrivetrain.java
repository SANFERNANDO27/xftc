package org.mooncat.xftc.subsystems.drivetrain;

import org.joml.Vector2d;
import org.mooncat.xftc.components.motors.XMotorController;
import org.mooncat.xftc.components.sensors.imu.XIMU;

import static java.lang.Math.sin;
import static java.lang.Math.cos;

public class MeccanumDrivetrain {
    XMotorController frontLeft;
    XMotorController frontRight;
    XMotorController backLeft;
    XMotorController backRight;
    XIMU imu;

    public MeccanumDrivetrain(XMotorController frontLeft, XMotorController frontRight, XMotorController backLeft, XMotorController backRight, XIMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
    }

    public void absoluteDrive(Vector2d velocity, double angularVelocity) {
        Vector2d absoluteVelocity = velocity.absolute();
        double normalizer = Math.max(absoluteVelocity.x() + absoluteVelocity.y() + angularVelocity, 1);

        // Diagonal LrlR
        frontLeft.setPower((velocity.y + velocity.x + angularVelocity) / normalizer);
        backRight.setPower((velocity.y + velocity.x - angularVelocity) / normalizer);

        // Diagonal lRLr
        frontRight.setPower((velocity.y - velocity.x - angularVelocity) / normalizer);
        backLeft.setPower((velocity.y - velocity.x + angularVelocity) / normalizer);
    }

    public void fieldCentricDrive(Vector2d velocity, double angularVelocity) {
        double heading = imu.getZAngleRad();
        Vector2d rotatedVelocity = new Vector2d(
                velocity.x * cos(-heading) - velocity.y * sin(-heading),
                velocity.x * sin(-heading) - velocity.y * cos(-heading)
        );

        Vector2d absoluteVelocity = velocity.absolute();
        double normalizer = Math.max(absoluteVelocity.x() + absoluteVelocity.y() + angularVelocity, 1);

        // Diagonal LrlR
        frontLeft.setPower((rotatedVelocity.y + rotatedVelocity.x + angularVelocity) / normalizer);
        backRight.setPower((rotatedVelocity.y + rotatedVelocity.x - angularVelocity) / normalizer);

        // Diagonal lRLr
        frontRight.setPower((rotatedVelocity.y - rotatedVelocity.x - angularVelocity) / normalizer);
        backLeft.setPower((rotatedVelocity.y - rotatedVelocity.x + angularVelocity) / normalizer);
    }
}
