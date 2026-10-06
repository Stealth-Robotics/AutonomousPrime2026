package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class Pose3D {
    private final double x;
    private final double y;
    private final double z;
    private final double heading;

    public Pose3D(double x, double y, double z, double heading) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.heading = heading;
    }

    public Pose3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.heading = 0;
    }

    public Pose3D(Pose pose, double z) {
        this(pose.x(), pose.y(), z, pose.heading());
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double z() {
        return z;
    }

    public double heading() {
        return heading;
    }
}