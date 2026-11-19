package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShotTimer {
    private final ElapsedTime timer = new ElapsedTime();

    private double elapsed;
    private boolean running;

    public void start() {
        elapsed = 0;
        timer.reset();
        running = true;
    }

    public void pause() {
        if (running) {
            elapsed += timer.seconds();
            running = false;
        }
    }

    public void resume() {
        if (!running) {
            timer.reset();
            running = true;
        }
    }

    public boolean isPaused() {
        return running = false;
    }

    public double seconds() {
        return elapsed + (running ? timer.seconds() : 0);
    }

    public boolean isFinished(double duration) {
        return seconds() >= duration;
    }

    public void reset() {
        elapsed = 0;
        running = false;
    }
}