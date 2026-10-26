package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.stealthrobotics.library.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.math.Pose;

import org.stealthrobotics.library.Alliance;

// Class designed to relocalize the robot on the field
public class Localizer {

    // Poses named from the drive station point of view, with robot facing towards opposing color's side
    private static final Pose RED_RIGHT_RESET_POSE = new Pose(8,8,0);
    private static final Pose RED_LEFT_RESET_POSE = new Pose(136, 8, 0);
    private static final Pose BLUE_RIGHT_RESET_POSE = new Pose(136,136,180);
    private static final Pose BLUE_LEFT_RESET_POSE = new Pose(8,136,180);


    public static Command resetPoseNearRight(Drive drive) {
        return new ConditionalCommand(
                new InstantCommand(() -> drive.setPose(RED_RIGHT_RESET_POSE)),
                new InstantCommand(() -> drive.setPose(BLUE_RIGHT_RESET_POSE)),
                Alliance::isRed
        );
    }
    public static Command resetPoseNearLeft(Drive drive) {
        return new ConditionalCommand(
                new InstantCommand(() -> drive.setPose(RED_LEFT_RESET_POSE)),
                new InstantCommand(() -> drive.setPose(BLUE_LEFT_RESET_POSE)),
                Alliance::isRed
        );
    }
    public static Command resetPoseFarRight(Drive drive) {
        return new ConditionalCommand(
                new InstantCommand(() -> drive.setPose(BLUE_LEFT_RESET_POSE)),
                new InstantCommand(() -> drive.setPose(RED_LEFT_RESET_POSE)),
                Alliance::isRed
        );
    }
    public static Command resetPoseFarLeft(Drive drive) {
        return new ConditionalCommand(
                new InstantCommand(() -> drive.setPose(BLUE_RIGHT_RESET_POSE)),
                new InstantCommand(() -> drive.setPose(RED_RIGHT_RESET_POSE)),
                Alliance::isRed
        );
    }
}
