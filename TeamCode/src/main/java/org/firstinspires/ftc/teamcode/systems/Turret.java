package org.firstinspires.ftc.teamcode.systems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private boolean isPositionLocked = false;

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, 0, 0, 0);

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
    }

    public void updateTracking(Pose robotPose, Pose3D targetCellPose) {
    }

    @Override
    public void periodic() {
    }
}
