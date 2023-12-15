package org.mooncat.xftc.mechanisms.drive

import org.joml.Vector2d
import org.mooncat.xftc.control.PID
import org.mooncat.xftc.mechanisms.Mechanism

/**
 * A common interface for drive mechanisms to move
 *
 * All drive mechanisms point towards the positive-x axis
 */
abstract class DriveMechanism : Mechanism {
    abstract val rotationPID: PID

    override fun update() {
        rotationPID.update()
    }

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
    abstract fun drive(velocity: Vector2d, angularVelocity: Double)

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
    abstract fun fieldOrientedDrive(velocity: Vector2d, angularVelocity: Double)

    /**
     * Sets the PID components for the integrated rotation PID
     *
     * @param[kp] The proportional term
     * @param[ki] The integral term
     * @param[kd] The derivative term
     */
    fun setRotationPIDParameters(kp: Double, ki: Double, kd: Double) {
        rotationPID.kp = kp
        rotationPID.ki = ki
        rotationPID.kd = kd
    }

    /**
     * Sets the target angle for the rotation PID
     *
     * @param[radians] The angle, in radians. Measured counter-clockwise
     */
    fun setRotationAngle(radians: Double) {
        rotationPID.setpoint = radians
    }

    /**
     * Enables rotating the robot via PID calculations
     */
    fun enableRotation() {
        rotationPID.enable()
    }

    /**
     * Disables rotating the robot via PID calculations
     */
    fun disableRotation() {
        rotationPID.disable()
    }

}