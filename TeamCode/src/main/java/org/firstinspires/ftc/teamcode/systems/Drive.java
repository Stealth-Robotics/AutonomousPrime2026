package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Drive extends StealthSubsystem {
    private final Follower follower;

    public enum PoseResetLocation {
        FAR_LEFT_CORNER(
            new Pose(8.32,6.5, Math.toRadians(180)),
            new Pose(133,135.5, Math.toRadians(0))
        ),
        FAR_RIGHT_CORNER(
            new Pose(8.32,135.5, Math.toRadians(180)),
            new Pose(133,6.5, Math.toRadians(0))
        ),
        CLOSE_LEFT_CORNER(
            new Pose(133,6.5, Math.toRadians(180)),
            new Pose(8.32,135.5, Math.toRadians(0))
        ),
        CLOSE_RIGHT_CORNER(
            new Pose(133,135.5, Math.toRadians(180)),
            new Pose(8.32,6.5, Math.toRadians(0))
        );

        private final Pose bluePose, redPose;

        PoseResetLocation(Pose bluePose, Pose redPose) {
            this.bluePose = bluePose;
            this.redPose = redPose;
        }

        public Pose getPose() {
            return Alliance.isRed() ? bluePose : redPose;
        }
    }

    public Drive(HardwareMap h) {
        follower = Constants.create(h);
    }

    public Command teleopDrive(GamepadEx gamepad) {
        return infinite(() -> {
            if (gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
                follower.setHeading(Alliance.isRed() ? 0 : Math.toRadians(180));

            follower.manual(ManualDrive.fieldCentric(
                -gamepad.getLeftY(),
                gamepad.getLeftX(),
                -gamepad.getRightX(),
                follower.pose().heading()
            ));
        });
    }

    /*
     Resets the follower's pose to the indicated field corner
     (assumes the robot is pointing forward for whichever alliance we are on).
    */
    public Command resetPoseTo(PoseResetLocation resetLocation) {
        return instant(() -> follower.setPose(resetLocation.getPose()));
    }

    public Follower getFollower() {
        return follower;
    }

    public void setPose(Pose p) {
        follower.setPose(p);
    }

    public Pose getPose() {
        return follower.pose();
    }

    @Override
    public void periodic() {
        follower.update();

        telemetry.addData("Pose", "(" + follower.pose().x() + ", " +
            follower.pose().y() + ", " + Math.toDegrees(follower.pose().heading()) + ")");
    }
}
