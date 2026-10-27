package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.TurretController;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final DcMotorEx turretMotor;

    private final TurretController turretController;

    private final double MAX_TURRET_ANGLE_DEGREES = 200;
    private final double MIN_TURRET_ANGLE_DEGREES = -200;

    private final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private final double TICKS_PER_REVOLUTION = 384.5;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");

        turretController = new TurretController(
            0, 0, 0,
            MIN_TURRET_ANGLE_DEGREES,
            MAX_TURRET_ANGLE_DEGREES
        );

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getAngle() {
        return turretMotor.getCurrentPosition()
            * 360
            / TICKS_PER_REVOLUTION
            / TURRET_GEAR_RATIO;
    }

    public void update(double targetAngle, double targetVelocity) {
        this.targetAngle = targetAngle + 90; // 0° -> Forwards, 90° -> Left, 180° -> Back, 270° -> Right
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void periodic() {
//        turretMotor.setPower(turretController.calculate(getAngle(), targetAngle, targetVelocity));

        telemetry.addData("Turret Angle Degrees", getAngle());
    }
}
