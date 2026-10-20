package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Pose3D;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    private final NextMotor turretMotor = new NextMotor(RobotController.controlHub(), 0);

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, 0, 0, 0);

    // Returns a field relative turret offset by rotating the turret offset about the origin by the bot's heading.
    public static Pose3D turretOffsetWithAngle(double theta) {
        double radius = Math.sqrt(ROBOT_TO_TURRET_OFFSET.x() * ROBOT_TO_TURRET_OFFSET.x() + ROBOT_TO_TURRET_OFFSET.y() * ROBOT_TO_TURRET_OFFSET.y());
        double angle = Math.atan2(ROBOT_TO_TURRET_OFFSET.y(), ROBOT_TO_TURRET_OFFSET.x());
        return new Pose3D(
                radius * Math.cos(angle + theta),
                radius * Math.sin(angle + theta),
                ROBOT_TO_TURRET_OFFSET.z(),
                ROBOT_TO_TURRET_OFFSET.heading() + theta
        );
    }

    /*
        Add sotm to the turret. Use velocity compensdtion, turret tangential velocity compensation, tilt compensation, latency compensation
        A tuned rpm map and use a velocity ratio to scale it to account for radial velocity, calculate hood angle based on target trajectory.
        Calculate the trajectory by making sure that the ball first hits the top side of the hive cell first and use that as the shot
        constraint
     */

    public Turret() {
        turretMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(NextMotor.Direction.FORWARD);
    }

    public void updateTracking(Pose robotPose, Pose3D targetCellPose) {
    }

    @Override
    public void periodic() {
    }
}
