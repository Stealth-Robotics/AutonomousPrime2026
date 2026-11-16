package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class LoopTimeTracker {
    private long lastLoopTime = System.nanoTime();

    private double maxLoopTime;
    private double totalLoopTime;

    private int loopCount;
    private int slowLoops;
    private int verySlowLoops;

    public void update() {
        long now = System.nanoTime();

        double loopTime = (now - lastLoopTime) / 1000000.0;
        lastLoopTime = now;

        maxLoopTime = Math.max(maxLoopTime, loopTime);
        totalLoopTime += loopTime;
        loopCount++;

        if (loopTime > 20) slowLoops++;
        if (loopTime > 50) verySlowLoops++;

        telemetry.addData("Loop Time", "%.2f ms", loopTime);
        telemetry.addData("Loop Avg", "%.2f ms", totalLoopTime / loopCount);
        telemetry.addData("Loop Max", "%.2f ms", maxLoopTime);
        telemetry.addData("Slow Loops (>20ms)", slowLoops);
        telemetry.addData("Very Slow Loops (>50ms)", verySlowLoops);
    }
}