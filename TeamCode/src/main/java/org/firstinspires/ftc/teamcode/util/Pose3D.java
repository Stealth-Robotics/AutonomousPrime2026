package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;


public class Pose3D{
    private final double z;
    private final Pose pose;

    public Pose3D(double x, double y, double z, double heading) {
        this.pose = new Pose(x, y, heading);
        this.z = z;
    }

    public Pose3D(double x, double y, double z) {
        this.pose = new Pose(x, y);
        this.z = z;
    }

    public Pose3D(Pose pose, double z) {
        this(pose.x(), pose.y(), z, pose.heading());
    }

    public Pose3D plus(Pose3D other) {
        return new Pose3D(this.asPose().plus(other.asPose()), this.z() + other.z());
    }
    public Pose3D minus(Pose3D other) {
        return new Pose3D(this.asPose().minus(other.asPose()), this.z() - other.z());
    }

    // From https://en.wikipedia.org/wiki/Rotation_matrix - "In two dimensions"
    public Pose3D rotateBy(double degreeIncrement) {
        double theta = Math.toRadians(degreeIncrement);
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        return new Pose3D(this.x() * cos - this.y() * sin, this.y() * cos + this.x() * sin, this.z(), this.heading() + degreeIncrement);
    }

    public double x() {
        return pose.x();
    }

    public double y() {
        return pose.y();
    }

    public double z() {
        return z;
    }

    public double heading() {
        return pose.heading();
    }

    public Pose asPose() {
        return pose;
    }
}