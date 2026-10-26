package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.stealthrobotics.library.Alliance;

public class Targets {
    //TODO: find real poses
    public static Pose3D RED_RIGHT_TARGET = new Pose3D(58,64,64);
    public static Pose3D RED_LEFT_TARGET = new Pose3D(86, 64, 64);
    public static Pose3D BLUE_RIGHT_TARGET = new Pose3D(86,80,64);
    public static Pose3D BLUE_LEFT_TARGET = new Pose3D(58, 80, 64);

    public static Pose3D getTarget(Pose robotPose) {
        if(Alliance.isRed()) {
            return (robotPose.y() < 72 ? RED_RIGHT_TARGET : RED_LEFT_TARGET);
        } else {
            return (robotPose.y() > 72 ? BLUE_RIGHT_TARGET : BLUE_LEFT_TARGET);
        }
    }
}
