package org.mooncat.xftc.mechanisms.drive

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.joml.Vector2d
import org.mooncat.xftc.control.PID
import org.mooncat.xftc.util.DEFAULT_BACK_LEFT_MOTOR_PATH
import org.mooncat.xftc.util.DEFAULT_BACK_RIGHT_MOTOR_PATH
import org.mooncat.xftc.util.DEFAULT_FRONT_LEFT_MOTOR_PATH
import org.mooncat.xftc.util.DEFAULT_FRONT_RIGHT_MOTOR_PATH
import org.mooncat.xftc.util.DEFAULT_IMU_PATH
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * A drive mechanism which uses a mecanum drive chassis to move
 *
 * @property[frontLeftMotor] The front left motor as a [DcMotorEx]
 * @property[frontRightMotor] The front right motor as a [DcMotorEx]
 * @property[backLeftMotor] The back left motor as a [DcMotorEx]
 * @property[backRightMotor] The back right motor as a [DcMotorEx]
 * @property[imu] The imu to use for rotation calculations
 * @constructor Receives all the necessary motors and the IMU
 */
class MecanumDriveMechanism(private val frontLeftMotor: DcMotorEx,
                            private val frontRightMotor: DcMotorEx,
                            private val backLeftMotor: DcMotorEx,
                            private val backRightMotor: DcMotorEx,
                            private val imu: IMU) : DriveMechanism() {

    var angularSpeed = 1.0
    var speed = 1.0

    /**
     * Given an [OpMode], utilizes its hardware map to locate the necessary components with the
     * library's defaults
     */
    constructor(opMode: OpMode) : this(
            opMode.hardwareMap.get(DcMotorEx::class.java, DEFAULT_FRONT_LEFT_MOTOR_PATH),
            opMode.hardwareMap.get(DcMotorEx::class.java, DEFAULT_FRONT_RIGHT_MOTOR_PATH),
            opMode.hardwareMap.get(DcMotorEx::class.java, DEFAULT_BACK_LEFT_MOTOR_PATH),
            opMode.hardwareMap.get(DcMotorEx::class.java, DEFAULT_BACK_RIGHT_MOTOR_PATH),
            opMode.hardwareMap.get(IMU::class.java, DEFAULT_IMU_PATH)
    )

    override val rotationPID = PID({ output, factor ->
        val power = min(1.0, max(-1.0, output)) * factor

        frontLeftMotor.power = -power
        frontRightMotor.power = power
        backLeftMotor.power = -power
        backRightMotor.power = power

    }, {
        return@PID imu.robotYawPitchRollAngles.getPitch(AngleUnit.RADIANS)
    }, outputFactor = angularSpeed)

    override fun drive(velocity: Vector2d, angularVelocity: Double) {
        val maxValue = max(velocity.x.absoluteValue + velocity.y.absoluteValue + angularVelocity.absoluteValue, 1.0)

        frontLeftMotor.power = (velocity.x + velocity.y + angularVelocity) / maxValue
        frontRightMotor.power = (velocity.x - velocity.y - angularVelocity) / maxValue
        backLeftMotor.power = (velocity.x - velocity.y + angularVelocity) / maxValue
        backRightMotor.power = (velocity.x + velocity.y - angularVelocity) / maxValue
    }

    override fun fieldOrientedDrive(velocity: Vector2d, angularVelocity: Double) {
        val negativeHeading = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        val cosTheta = cos(negativeHeading)
        val sinTheta = sin(negativeHeading)

        val rotatedVelocity = Vector2d(
                velocity.y * cosTheta - velocity.x * sinTheta,
                velocity.y * sinTheta - velocity.x * cosTheta
        )

        val maxValue = max(rotatedVelocity.x.absoluteValue + rotatedVelocity.y.absoluteValue + angularVelocity.absoluteValue, 1.0)

        frontLeftMotor.power = (rotatedVelocity.x + rotatedVelocity.y + angularVelocity) / maxValue
        frontRightMotor.power = (rotatedVelocity.x - rotatedVelocity.y - angularVelocity) / maxValue
        backLeftMotor.power = (rotatedVelocity.x - rotatedVelocity.y + angularVelocity) / maxValue
        backRightMotor.power = (rotatedVelocity.x + rotatedVelocity.y - angularVelocity) / maxValue
    }

}