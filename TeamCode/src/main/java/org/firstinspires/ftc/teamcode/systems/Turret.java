package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TurretController;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final TurretController turretController;

    private static final double MAX_TURRET_POWER = 0.6;

    private static final double MAX_TURRET_ANGLE_DEGREES = 450;
    private static final double MIN_TURRET_ANGLE_DEGREES = 90;

    private static final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");
        turretController = new TurretController(
            0.1, 0, 0,
            MIN_TURRET_ANGLE_DEGREES,
            MAX_TURRET_ANGLE_DEGREES
        );

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getAngle() {
        return 180 + (turretMotor.getCurrentPosition() * 360 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO);
    }

    public void update(double targetAngle, double targetVelocity) {
        this.targetAngle = targetAngle;
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void periodic() {
        double currentAngle = getAngle();

        double output = turretController.calculate(
            currentAngle,
            targetAngle,
            targetVelocity
        );

        output = MathUtils.clamp(output, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        turretMotor.setPower(output);

        telemetry.addData("Turret Power", turretMotor.getPower());
        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData("Turret Error", turretController.getPositionError(currentAngle, targetAngle));
    }
}
