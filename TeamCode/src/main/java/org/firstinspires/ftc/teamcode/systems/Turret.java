package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDFController turretPIDF = new PIDFController(0,0,0,0);

    private final double MAX_TURRET_ANGLE_DEGREES = 200;
    private final double MIN_TURRET_ANGLE_DEGREES = -200;

    private final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private final double TICKS_PER_REVOLUTION = 384.5;

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getAngle() {
        return turretMotor.getCurrentPosition() * TICKS_PER_REVOLUTION * TURRET_GEAR_RATIO * 360.0;
    }

    public void updateTracking(ShotCalculator shotCalculator) {
        double targetTurretAngle = shotCalculator.getTurretAngle();
        turretPIDF.setSetPoint(shotCalculator.getTurretAngle());
    }

    @Override
    public void periodic() {
        turretMotor.setPower(turretPIDF.calculate(getAngle()));
    }
}
