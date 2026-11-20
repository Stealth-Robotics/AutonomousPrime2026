package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private final TurretState state = TurretState.TRACKING;

    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

    private final DcMotorEx turretMotor;
    private final PIDController turretController;

    private static final double TURRET_TOLERANCE_DEGREES = 5;

    private static final double MAX_TURRET_ANGLE_DEGREES = 270;
    private static final double MIN_TURRET_ANGLE_DEGREES = -90;

    private static final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;

    private static final double TURRET_kPV = 0;
    private static final double TURRET_kV = 0.002;
    private static final double TURRET_kS = 0.04;

    private static final double WRAP_DETECTION_DEGREES = 180;
    private static final double WRAP_HYSTERESIS_DEGREES = 5;

    private boolean wrapping = false;
    private double previousTargetAngle = Double.NaN;

    private double currentAngle, targetAngle, targetVelocity;

    private enum TurretState {
        HOMED,
        TRACKING
    }

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");
        turretController = new PIDController(0.01, 0, 0.0005);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double findValidTargetAngle(double desiredTargetAngle) {
        double bestAngle = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        int minK = (int) Math.ceil((MIN_TURRET_ANGLE_DEGREES - desiredTargetAngle) / 360.0);
        int maxK = (int) Math.floor((MAX_TURRET_ANGLE_DEGREES - desiredTargetAngle) / 360.0);

        for (int k = minK; k <= maxK; k++) {
            double candidate = desiredTargetAngle + 360.0 * k;
            double distance = Math.abs(candidate - currentAngle);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
            }
        }

        return bestAngle;
    }

    public double getAngle() {
        return turretMotor.getCurrentPosition() * 360 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO;
    }

    public boolean atAngle() {
        return Math.abs(currentAngle - targetAngle) < TURRET_TOLERANCE_DEGREES;
    }

    private double getVelocity() {
        return turretMotor.getVelocity() * 360 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO;
    }

    public boolean isWrapping() {
        return wrapping;
    }

    public void update(double targetAngle, double targetVelocity) {
        if (state == TurretState.TRACKING) {
            double newTargetAngle = getHysteresisTarget(targetAngle + 180);

            if (!Double.isNaN(previousTargetAngle) && Math.abs(newTargetAngle - previousTargetAngle) > WRAP_DETECTION_DEGREES)
                wrapping = true;

            this.targetAngle = newTargetAngle;
            this.targetVelocity = targetVelocity;

            previousTargetAngle = newTargetAngle;

            if (wrapping && atAngle())
                wrapping = false;
        }
        else {
            this.targetAngle = 0;
            this.targetVelocity = 0;

            previousTargetAngle = Double.NaN;

            wrapping = false;
        }
    }

    private double getHysteresisTarget(double desiredTargetAngle) {
        double normalizedAngle = desiredTargetAngle;

        while (normalizedAngle < MIN_TURRET_ANGLE_DEGREES)
            normalizedAngle += 360;

        while (normalizedAngle >= MAX_TURRET_ANGLE_DEGREES)
            normalizedAngle -= 360;

        if (previousTargetAngle >= MAX_TURRET_ANGLE_DEGREES - WRAP_HYSTERESIS_DEGREES
            && normalizedAngle < MIN_TURRET_ANGLE_DEGREES + WRAP_HYSTERESIS_DEGREES)
            return MAX_TURRET_ANGLE_DEGREES;

        if (previousTargetAngle <= MIN_TURRET_ANGLE_DEGREES + WRAP_HYSTERESIS_DEGREES
            && normalizedAngle >= MAX_TURRET_ANGLE_DEGREES - WRAP_HYSTERESIS_DEGREES)
            return MIN_TURRET_ANGLE_DEGREES;

        return findValidTargetAngle(desiredTargetAngle);
    }

    @Override
    public void periodic() {
        currentAngle = getAngle();

        double motorPower = turretController.calculate(currentAngle, targetAngle);

        if (Math.abs(currentAngle - targetAngle) > 1)
            motorPower += TURRET_kS * Math.signum(motorPower);

        double velocityError = targetVelocity - getVelocity();
        motorPower += TURRET_kPV * velocityError + TURRET_kV * targetVelocity;

        turretMotor.setPower(motorPower);

        dashboard.addData("current angle", currentAngle);
        dashboard.addData("target angle", targetAngle);
        dashboard.addData("current velocity", getVelocity());
        dashboard.addData("target velocity", targetVelocity);
        dashboard.update();

        telemetry.addData("Turret Angle Error", turretController.getPositionError());
    }
}
