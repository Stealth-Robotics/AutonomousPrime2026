package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class LoopTimeTracker {
    private long lastLoopTime = System.nanoTime();

    public void update() {
        long now = System.nanoTime();

        double loopTime = (now - lastLoopTime) / 1000000.0;
        lastLoopTime = now;

        telemetry.addData("Loop Time", "%.2f ms", loopTime);
    }
}