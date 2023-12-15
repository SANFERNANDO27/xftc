package org.mooncat.xftc.control

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.absoluteValue

class PID(private val feedAction: (output: Double, factor: Double) -> Unit,
          private val readAction: () -> Double,
          kp: Double = 0.1,
          ki: Double = 0.0,
          kd: Double = 0.0,
          var tolerance: Double = 0.01,
          var setpoint: Double = 0.0,
          var outputFactor: Double = 1.0) {

    var kp: Double = kp
        set(value) {
            if (value < 0.0) {
                throw IllegalArgumentException("PID Kp coefficient cannot be negative")
            }

            field = value
        }

    var ki: Double = ki
        set(value) {
            if (value < 0.0) {
                throw IllegalArgumentException("PID Ki coefficient cannot be negative")
            }

            field = value
        }

    var kd: Double = kd
        set(value) {
            if (value < 0.0) {
                throw IllegalArgumentException("PID Kd coefficient cannot be negative")
            }

            field = value
        }

    private val timer = ElapsedTime()
    private var integralSum = 0.0
    private var lastError = 0.0

    private var enabled = false
    private var disableWhenSetpointReached = false

    private var firstRun = false

    fun update() {
        if (!enabled) return

        val current = readAction()
        if (isAtSetpoint(current)) {
            if (disableWhenSetpointReached) {
                disable()
            }
            return
        };

        val output = calculate(setpoint, current)
        feedAction(output, outputFactor)
    }

    private fun calculate(current: Double, reference: Double = setpoint): Double {
        if (firstRun) {
            timer.reset()
        }

        val seconds = timer.seconds()

        val error = reference - current
        val derivative = (error + lastError) / seconds

        integralSum += error * seconds

        lastError = error
        timer.reset()

        return (kp * error) + (ki * integralSum) + (kd * derivative)
    }

    fun isAtSetpoint(current: Double): Boolean {
        return current.absoluteValue <= setpoint.absoluteValue + tolerance
    }

    fun enable() {
        enabled = true
    }

    fun disable() {
        enabled = false
    }

    fun shouldDisableWhenSetpointReached(switch: Boolean) {
        this.disableWhenSetpointReached = true
    }

}