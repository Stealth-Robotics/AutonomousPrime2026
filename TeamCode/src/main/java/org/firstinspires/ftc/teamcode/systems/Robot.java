package org.firstinspires.ftc.teamcode.systems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private static final Pose BLUE_RESET_POSE = new Pose(0, 0, 0);
    private static final Pose RED_RESET_POSE = new Pose(0, 0, 0);

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public void resetPose() {
        if (Alliance.get().equals(Alliance.BLUE))
            drive.setPose(BLUE_RESET_POSE);
        else
            drive.setPose(RED_RESET_POSE);
    }

    @Override
    public void periodic() {
    }
}
