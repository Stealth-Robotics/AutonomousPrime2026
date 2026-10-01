package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.StealthSubsystem;

public class Drive extends StealthSubsystem {
    private final Follower pedroFollower;

    public Drive(HardwareMap hardwareMap) {
        pedroFollower = Constants.create(hardwareMap);
    }

    public void driveFieldCentric(double x, double y, double rot) {
        pedroFollower.manual(ManualDrive.fieldCentric(x, y, rot, pedroFollower.pose().heading()));
    }

    @Override
    public void periodic() {
        pedroFollower.update();

        telemetry.addData("Follower Mode: ", pedroFollower.mode());
    }
}
