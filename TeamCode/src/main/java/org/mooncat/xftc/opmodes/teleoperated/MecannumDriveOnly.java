package org.mooncat.xftc.opmodes.teleoperated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.joml.Vector2d;
import org.mooncat.xftc.components.motors.MotorController;
import org.mooncat.xftc.components.motors.XMotorController;
import org.mooncat.xftc.components.sensors.imu.RevHubIMU;
import org.mooncat.xftc.components.sensors.imu.XIMU;
import org.mooncat.xftc.opmodes.XOpMode;
import org.mooncat.xftc.state.StaticRobotState;
import org.mooncat.xftc.subsystems.drivetrain.MeccanumDrivetrain;
import org.mooncat.xftc.util.StringConstants;

@TeleOp
public class MecannumDriveOnly extends XOpMode {
    XIMU imu;
    MeccanumDrivetrain drivetrain;

    @Override
    protected void initialize() {
        imu = new RevHubIMU(hardwareMap, new IMU.Parameters(StaticRobotState.HUB_ORIENTATION));
        drivetrain = new MeccanumDrivetrain(
                new MotorController(hardwareMap, StringConstants.FRONT_LEFT_MOTOR_PATH),
                new MotorController(hardwareMap, StringConstants.FRONT_RIGHT_MOTOR_PATH),
                new MotorController(hardwareMap, StringConstants.BACK_LEFT_MOTOR_PATH),
                new MotorController(hardwareMap, StringConstants.BACK_RIGHT_MOTOR_PATH),
                imu
        );

    }

    @Override
    protected void periodic() {
        Vector2d velocity = new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double angularVelocity = gamepad1.right_stick_x;

        drivetrain.fieldCentricDrive(velocity, angularVelocity);
    }

    @Override
    protected void finalize_() {

    }
}
