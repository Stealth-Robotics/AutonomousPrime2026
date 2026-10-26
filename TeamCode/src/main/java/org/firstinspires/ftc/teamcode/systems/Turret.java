package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AnglePIDFController;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final AnglePIDFController turretController;

    private final double MAX_TURRET_ANGLE_DEGREES = 200;
    private final double MIN_TURRET_ANGLE_DEGREES = -200;

    private final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private final double TICKS_PER_REVOLUTION = 384.5;

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");
        turretController = new AnglePIDFController(0.0, 0.0, 0.0, 0.0);
        turretController.setAngularParams(-200, 200, 360);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getAngle() {
        return turretMotor.getCurrentPosition() * TICKS_PER_REVOLUTION * TURRET_GEAR_RATIO * 360.0;
    }

    public void updateTracking(ShotCalculator shotCalculator) {
        setTrigAngle(shotCalculator.getTurretAngle());
    }

    /**
     * Set the angle as the output of trigonometric calculations. DO NOT USE AS setSetPoint()!!!!
     * <a href="https://en.wikipedia.org/wiki/Angle#Signed_angles">...</a>
     */
    private void setTrigAngle(double degrees) {
        setSetPoint(degrees + 90);
    }

    /**
     * Sets the setpoint to the desired angle in degrees. NOTE: 0° -> Forwards, 90° -> Left, 180° -> Back, 270° -> Right
     */
    public void setSetPoint(double degrees) {
        turretController.setSetPoint(degrees);
    }

    @Override
    public void periodic() {
        turretMotor.setPower(turretController.calculate(getAngle()));
    }
}
