package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class Turret extends StealthSubsystem {
    private TurretState state = TurretState.HOMED;

    private final DcMotorEx turretMotor;
    private final PIDController turretController;

    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

    private static final double TURRET_TOLERANCE_DEGREES = 5;

    private static final double MAX_TURRET_ANGLE_DEGREES = 270;
    private static final double MIN_TURRET_ANGLE_DEGREES = -90;

    private static final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;

    private static final double TURRET_KS = 0.065;
    private static final double TURRET_KV = 0.0012;

    private double currentAngle, targetAngle, targetVelocity, lastMotorPower;

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

    public Command setTracking() {
        return new InstantCommand(() -> {
            state = TurretState.TRACKING;
            turretController.reset();
        });
    }

    public Command setHomed() {
        return new InstantCommand(() -> state = TurretState.HOMED);
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

    public void update(double targetAngle, double targetVelocity) {
        if (state == TurretState.TRACKING) {
            this.targetAngle = findValidTargetAngle(targetAngle + 180);
            this.targetVelocity = targetVelocity;
        }
        else {
            this.targetAngle = 0;
            this.targetVelocity = 0;
        }
    }

    @Override
    public void periodic() {
        currentAngle = getAngle();

        double motorPower = turretController.calculate(currentAngle, targetAngle);

        if (Math.abs(currentAngle - targetAngle) > 1)
            motorPower += TURRET_KS * Math.signum(motorPower);

        motorPower += TURRET_KV * targetVelocity;

        //Power deadband for optimal hardware writes
        if (Math.abs(motorPower - lastMotorPower) > 0.01 || motorPower == 0) {
            turretMotor.setPower(motorPower);

            lastMotorPower = motorPower;
        }

        dashboard.addData("Current Angle", currentAngle);
        dashboard.addData("Target Angle", targetAngle);
        dashboard.update();

        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Velocity", turretMotor.getVelocity() * 360 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO);
        telemetry.addData("Turret Angle Error", turretController.getPositionError());
    }
}
