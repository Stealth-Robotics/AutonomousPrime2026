package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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

import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    private TurretState state = TurretState.TRACKING;

    private final DcMotorEx turretMotor;
    private final PIDController turretController;

    private final VoltageSensor voltageSensor;

    private static final double TURRET_TOLERANCE_DEGREES = 2;

    private static final double MAX_MOTOR_POWER = 0.6;

    private static final double MAX_TURRET_ANGLE_DEGREES = 270;
    private static final double MIN_TURRET_ANGLE_DEGREES = -90;

    private static final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;

    private static final double TURRET_KS = 0.06;

    private double currentAngle, targetAngle;

    private enum TurretState {
        HOMED,
        TRACKING
    }

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");
        turretController = new PIDController(0.005, 0, 0);

        voltageSensor = h.voltageSensor.iterator().next();

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

    public void update(double targetAngle) {
        this.targetAngle =
            (state.equals(TurretState.TRACKING)) ? findValidTargetAngle(targetAngle + 180) : 0;
    }

    @Override
    public void periodic() {
        currentAngle = getAngle();

        double output = turretController.calculate(currentAngle, targetAngle);

        if (Math.abs(currentAngle - targetAngle) > 1)
            output += TURRET_KS * Math.signum(output);

        //Normalize the output to 12V even when the battery voltage is higher
        output *= 12.0 / voltageSensor.getVoltage();
        output = MathUtils.clamp(output, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

        turretMotor.setPower(output);

        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Angle Error", turretController.getPositionError());
    }
}
