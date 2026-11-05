package org.firstinspires.ftc.teamcode.util;

public class LowPassFilter {
    private double previous;
    private double alpha;
    public LowPassFilter(double alpha) {
        this.alpha = alpha;
        previous = 0;
    }
    public void update(double newValue) {
        previous = newValue * alpha + previous * (1 - alpha);
    }
    public double get() {
        return previous;
    }
}
