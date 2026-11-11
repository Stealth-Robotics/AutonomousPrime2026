package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.TurretController;
import org.stealthrobotics.library.StealthSubsystem;

public class Turret extends StealthSubsystem {
    // Allows us to freeze the turret on startup for debugging
    private final TurretState state = TurretState.HOMED;

    private final DcMotorEx turretMotor;
    private final TurretController turretController;

    private static final double MAX_TURRET_POWER = 0.8;

    private static final double MAX_TURRET_ANGLE_DEGREES = 270;
    private static final double MIN_TURRET_ANGLE_DEGREES = -90;

    private static final double TURRET_ANGLE_TOLERANCE = 5;

    private static final double TURRET_GEAR_RATIO = 100.0 / 30.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;

    private static final double WRAP_LOOKAHEAD_MS = 250;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    public enum TurretState {
        TRACKING,
        HOMED
    }

    public Turret(HardwareMap h) {
        turretMotor = h.get(DcMotorEx.class, "turret");
        turretController = new TurretController(
            0.01, 0, 0,
            MIN_TURRET_ANGLE_DEGREES,
            MAX_TURRET_ANGLE_DEGREES
        );

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Command resetToZero() {
        return new InstantCommand(() -> {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    public boolean isNotWrapping() {
        double currentAngle = getAngle();
        double velocity = turretMotor.getVelocity() * DEGREES_PER_TICK;
        double projectedAngle = currentAngle + velocity * (WRAP_LOOKAHEAD_MS / 1000.0);

        return projectedAngle > MIN_TURRET_ANGLE_DEGREES
            && projectedAngle < MAX_TURRET_ANGLE_DEGREES
            && currentAngle > MIN_TURRET_ANGLE_DEGREES
            && currentAngle < MAX_TURRET_ANGLE_DEGREES;
    }

    public double getAngle() {
        return turretMotor.getCurrentPosition() * 360 / TICKS_PER_REVOLUTION / TURRET_GEAR_RATIO;
    }

    public void update(double targetAngle, double targetVelocity) {
        this.targetAngle = state.equals(TurretState.TRACKING) ? targetAngle + 180 : 0;
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void periodic() {
        double currentAngle = getAngle();

        double output = turretController.calculate(currentAngle, targetAngle, targetVelocity);
        turretMotor.setPower(MathUtils.clamp(output, -MAX_TURRET_POWER, MAX_TURRET_POWER));

        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Angle Error", turretController.getPositionError(currentAngle, targetAngle));
    }
}
