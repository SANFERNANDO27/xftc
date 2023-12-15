package org.mooncat.xftc.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.joml.Vector2d
import org.mooncat.xftc.mechanisms.drive.MecanumDriveMechanism

class MecanumDriveOnly : OpMode() {
    private val mecanumDrivetrain = MecanumDriveMechanism(this)

    override fun init() {
    }

    override fun loop() {
        mecanumDrivetrain.drive(Vector2d(gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble()), gamepad1.right_stick_x.toDouble())
    }
}