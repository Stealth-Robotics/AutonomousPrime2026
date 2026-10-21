package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Pose3D;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    private final NextMotor turretMotor = new NextMotor(RobotController.controlHub(), 3);

    public Turret() {
        //Uncomment when using
//        turretMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setDirection(NextMotor.Direction.FORWARD);
    }

    public void updateTracking(Pose robotPose, Pose3D targetCellPose) {
    }

    @Override
    public void periodic() {
    }
}
