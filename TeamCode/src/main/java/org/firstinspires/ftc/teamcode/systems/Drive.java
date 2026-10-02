package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutoDataStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.StealthSubsystem;

public class Drive extends StealthSubsystem {
    private final Follower pedroFollower;

    public Drive(HardwareMap hardwareMap) {
        pedroFollower = Constants.create(hardwareMap);

        //Set the follower's pose to the end of auto pose (overwritten if still in auto)
        pedroFollower.setPose(AutoDataStorage.endOfAutoPose);
    }

    public void driveFieldCentric(double x, double y, double rot) {
        pedroFollower.manual(ManualDrive.fieldCentric(x, y, rot, pedroFollower.pose().heading()));
    }

    public void setPose(Pose p) {
        pedroFollower.setPose(p);
    }

    public Pose getPose() {
        return pedroFollower.pose();
    }

    @Override
    public void periodic() {
        pedroFollower.update();

        telemetry.addData("Follower Mode: ", pedroFollower.mode());
    }
}
