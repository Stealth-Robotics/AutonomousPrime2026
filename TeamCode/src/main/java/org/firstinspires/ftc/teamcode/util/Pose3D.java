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

    public Pose3D plus(Pose3D other) {
        return new Pose3D(this.x + other.x, this.y + other.y, this.z + other.z, this.heading + other.heading);
    }
    public Pose3D minus(Pose3D other) {
        return new Pose3D(this.x - other.x, this.y - other.y, this.z - other.z, this.heading - other.heading);
    }
    public Pose3D rotate(double dHeading) {
        return new Pose3D(this.x, this.y, this.z, this.heading + dHeading);
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