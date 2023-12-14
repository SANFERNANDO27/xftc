package org.mooncat.xftc.mechanisms.drive

import org.joml.Vector2d
import org.mooncat.xftc.control.PID

interface DriveMechanism {
    val rotationPID: PID

    /**
     * Causes this drive to move in the desired direction relative to the robot's own position
     * and rotation
     *
     * **Note:** Drive mechanisms which do not support sideways movement will not benefit
     * from the velocity's y-component
     *
     * @param[velocity] The velocity vector, where x is forward and y is sideways
     * @param[angularVelocity] The angular velocity, counter-clockwise
     */
    fun drive(velocity: Vector2d, angularVelocity: Double)

    /**
     * Causes this drive to move in the desired direction relative to its field position and
     * rotation
     *
     * **Note:** Drive mechanisms which do not support sideways movement will not benefit
     * from the velocity's y-component
     *
     * @param[velocity] The velocity vector, where x is forward and y is sideways
     * @param[angularVelocity] The angular velocity, counter-clockwise
     */
    fun fieldOrientedDrive(velocity: Vector2d, angularVelocity: Double)

    fun rotate(radians: Double, speed: Double)

    fun setRotationPIDParameters(kp: Double, ki: Double, kd: Double) {
        rotationPID.kp = kp
        rotationPID.ki = ki
        rotationPID.kd = kd
    }

}