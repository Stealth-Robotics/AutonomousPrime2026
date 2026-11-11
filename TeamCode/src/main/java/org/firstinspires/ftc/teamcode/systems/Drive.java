package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Drive extends StealthSubsystem {
    private final Follower follower;
    private final LowPassFilter vx, vy, vOmega;

    public enum PoseResetLocation {
        NEAR_LEFT_CORNER(
            new Pose(134.5,6.5, Math.toRadians(180)),
            new Pose(7,135.5, Math.toRadians(0))
        ),
        NEAR_RIGHT_CORNER(
            new Pose(134.5,135.5, Math.toRadians(180)),
            new Pose(7,6, Math.toRadians(0))
        ),

        LEFT_SIDE_OF_FLOWER(
            new Pose(134.5,85.5, Math.toRadians(180)),
            new Pose(7,56, Math.toRadians(0))
        ),
        RIGHT_SIDE_OF_FLOWER(
            new Pose(134.5,104.1, Math.toRadians(180)),
            new Pose(7,37.4, Math.toRadians(0))
        );

        private final Pose bluePose, redPose;

        PoseResetLocation(Pose bluePose, Pose redPose) {
            this.bluePose = bluePose;
            this.redPose = redPose;
        }

        public Pose getPose() {
            return Alliance.isRed() ? redPose : bluePose;
        }
    }

    public Drive(HardwareMap h) {
        follower = Constants.create(h);

        vx = new LowPassFilter(0.1);
        vy = new LowPassFilter(0.2);
        vOmega = new LowPassFilter(0.1);
    }

    public Command teleopDrive(GamepadEx gamepad) {
        return infinite(() -> {
            if (gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
                follower.setHeading(Alliance.isRed() ? 0 : Math.toRadians(180));

            follower.manual(ManualDrive.fieldCentric(
                -gamepad.getLeftY() * (Alliance.isRed() ? -1 : 1),
                gamepad.getLeftX() * (Alliance.isRed() ? -1 : 1),
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
        return new InstantCommand(() -> follower.setPose(resetLocation.getPose()));
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

    // Returns the field relative velocity of the robot
    public Pose3D getVelocity() {
        return new Pose3D(vx.get(), vy.get(), 0.0, vOmega.get());
    }

    public void updateVelocity() {
        Velocity v = follower.velocity();
        vx.update(v.vx);
        vy.update(v.vy);
        vOmega.update(v.omega);
    }

    @Override
    public void periodic() {
        follower.update();
        updateVelocity();

        telemetry.addData(
            "Pose", "(%.2f in, %.2f in, %.2f deg)",
            follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData(
            "Velocity", "(%.2f in/s, %.2f in/s, %.2f deg/s)",
            vx.get(), vy.get(), Math.toDegrees(vOmega.get())
        );
    }
}
