package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class Shooter extends StealthSubsystem {
    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;

    private final Servo gateServo;
    private final Servo hoodServo;

    private final PIDFController shooterPIDF = new PIDFController(
        0.0005, // 0.001
        0,
        0,
        0.00021
    );

    private ShooterState shooterState = ShooterState.IDLE;
    private GateState gateState = GateState.CLOSED;

    private static final double RPM_TOLERANCE = 50;

    private static final double HOOD_MIN_POSITION = 0;
    private static final double HOOD_MAX_POSITION = 0.3;

    private static final double HOOD_MIN_ANGLE = 2.5;
    private static final double HOOD_MAX_ANGLE = 22.5;

    private static final double GATE_OPEN_POSITION = 0.18;
    private static final double GATE_CLOSED_POSITION = 0.4;

    private static final double TICKS_PER_REVOLUTION = 28;

    private double targetRPM = 0;
    private double hoodAngle = HOOD_MIN_ANGLE;

    public enum GateState {
        OPEN,
        CLOSED
    }

    public enum ShooterState {
        SHOOTING,
        IDLE
    }

    public Shooter(HardwareMap h) {
        leftShooterMotor = h.get(DcMotorEx.class, "left_shooter");
        rightShooterMotor = h.get(DcMotorEx.class, "right_shooter");

        gateServo = h.get(Servo.class, "gate");
        hoodServo = h.get(Servo.class, "hood");

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodServo.setDirection(Servo.Direction.REVERSE);
    }

    public Command runShooter() {
        return instant(() -> {
            shooterState = ShooterState.SHOOTING;
        });
    }

    public Command stopShooter() {
        return instant(() -> {
            shooterState = ShooterState.IDLE;

            shooterPIDF.reset();

            leftShooterMotor.setPower(0.0);
            rightShooterMotor.setPower(0.0);
        });
    }

    public Command openGate() {
        return instant(() -> {
            gateServo.setPosition(GATE_OPEN_POSITION);
            gateState = GateState.OPEN;
        });
    }

    public Command closeGate() {
        return instant(() -> {
            gateServo.setPosition(GATE_CLOSED_POSITION);
            gateState = GateState.CLOSED;
        });
    }

    public void update(double rpm, double hoodAngle) {
        targetRPM = rpm;

        double angleDegrees = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, hoodAngle));
        double servoPosition = HOOD_MIN_POSITION + (angleDegrees - HOOD_MIN_ANGLE) /
            (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE) * (HOOD_MAX_POSITION - HOOD_MIN_POSITION);

        this.hoodAngle = angleDegrees;
        hoodServo.setPosition(servoPosition);
    }

    public boolean atTargetRPM() {
        return Math.abs(getCurrentRPM() - targetRPM) < RPM_TOLERANCE;
    }

    private double getCurrentRPM() {
        return rightShooterMotor.getVelocity() * 60 / TICKS_PER_REVOLUTION;
    }

    @Override
    public void periodic() {
        double currentRPM = getCurrentRPM();

        if (shooterState.equals(ShooterState.SHOOTING)) {
            double calculatedPower = shooterPIDF.calculate(currentRPM, targetRPM);

            leftShooterMotor.setPower(calculatedPower);
            rightShooterMotor.setPower(calculatedPower);

            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("velocity", rightShooterMotor.getVelocity());
        }

        dashboard.addData("current rpm", currentRPM);
        dashboard.addData("target rpm", targetRPM);
        dashboard.update();

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Hood Angle", hoodAngle);
    }
}
