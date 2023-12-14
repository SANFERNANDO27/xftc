package org.mooncat.xftc.control

import com.qualcomm.robotcore.util.ElapsedTime

class PID(var kp: Double = 0.0,
          var ki: Double = 0.0,
          var kd: Double = 0.0) {

    var setpoint = 0.0

    private val timer = ElapsedTime()
    private var integralSum = 0.0
    private var lastError = 0.0

    fun calculate(current: Double, reference: Double = setpoint): Double {
        val error = reference - current
        val seconds = timer.seconds()
        val derivative = (error - lastError) / seconds

        integralSum += error * seconds

        lastError = error
        timer.reset()

        return (kp * error) + (ki * integralSum) + (kd * derivative)
    }

}