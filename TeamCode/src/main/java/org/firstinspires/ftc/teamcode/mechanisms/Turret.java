package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.StealthSubsystem;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.units.measuretypes.Angle;

public class Turret extends StealthSubsystem {
    private final NextMotor turretMotor;

    private final Angle MAX_TURRET_ANGLE = Degrees.of(270);
    private final Angle MIN_TURRET_ANGLE = Degrees.of(-270);

    private final double TURRET_GEAR_RATIO = 100.0 / 30.0;

    public Turret(HardwareMap h) {
        turretMotor = h.get(NextMotor.class, "turret");
        turretMotor.setAnglePerCount(Degrees.of(360.0 / 384.5));

        turretMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(NextMotor.Direction.FORWARD);

        //Set the PIDF coefficients for the turret motor to use
        turretMotor.getPositionConstants()
            .withP(0)
            .withI(0)
            .withD(0)
            .withS(0);
    }

    public void updateTracking(Pose robotPose, Pose3D targetCellPose) {
    }

    private void setTurretAngle(Angle angle) {
        //Calculate the shortest path to the target angle without exceeding limits

        //Apply the target angle to the turret motor accounting for the gear ratio
        turretMotor.setPositionSetpoint(angle.times(TURRET_GEAR_RATIO));
    }

    @Override
    public void periodic() {
    }
}
