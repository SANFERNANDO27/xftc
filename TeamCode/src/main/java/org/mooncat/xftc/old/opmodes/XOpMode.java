package org.mooncat.xftc.old.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

public abstract class XOpMode extends LinearOpMode {

    protected abstract void initialize();
    protected abstract void periodic();
    protected abstract void finalize_();


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            periodic();
        }

        finalize_();
    }

}
