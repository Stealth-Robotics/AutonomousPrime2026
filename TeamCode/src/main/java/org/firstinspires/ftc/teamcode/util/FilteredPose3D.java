package org.firstinspires.ftc.teamcode.util;

public class FilteredPose3D { // Filtered pose for limelight readings - using a low-pass filter algorithm
    private Pose3D previous;
    private final double alpha;
    private int readings;

    public FilteredPose3D(double alpha) {
        this.alpha = alpha;
        previous = new Pose3D();
        readings = 0;
    }

    public void update(Pose3D newValue) {
        if (readings == 0) { // Give full weight to an initial value
            previous = newValue;
            readings++;
            return;
        }

        previous = new Pose3D(
                previous.x() * alpha + newValue.x() * (1 - alpha),
                previous.y() * alpha + newValue.y() * (1 - alpha),
                previous.z() * alpha + newValue.z() * (1 - alpha),
                previous.heading() * alpha + newValue.heading() * (1 - alpha)
        );
        readings++;
    }

    public Pose3D get() {
        return previous;
    }

    public int numReadings() {
        return readings;
    }

    public void clear() {
        previous = new Pose3D();
        readings = 0;
    }
}
