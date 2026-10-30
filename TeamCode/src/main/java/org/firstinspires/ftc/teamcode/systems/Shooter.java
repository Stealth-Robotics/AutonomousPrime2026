package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

public class Shooter extends StealthSubsystem {
    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;

    private final Servo gateServo;
    private final Servo hoodServo;

    private final PIDFController shooterPIDF = new PIDFController(
        0.0005,
        0,
        0,
        0.00021
    );

    private ShooterState state = ShooterState.IDLE;

    private static final double RPM_TOLERANCE = 50;

    private static final double HOOD_MIN_POSITION = 0.16;
    private static final double HOOD_MAX_POSITION = 0.3;

    private static final double HOOD_MIN_ANGLE = 11.127;
    private static final double HOOD_MAX_ANGLE = 20;

    private static final double GATE_OPEN_POSITION = 0.18;
    private static final double GATE_CLOSED_POSITION = 0.3;

    private static final double TICKS_PER_REVOLUTION = 28;

    private double targetRPM = 0;

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
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo.setDirection(Servo.Direction.REVERSE);
    }

    public Command runShooter() {
        return new InstantCommand(() -> state = ShooterState.SHOOTING);
    }

    public Command stopShooter() {
        return new InstantCommand(() -> state = ShooterState.IDLE);
    }

    public Command openGate() {
        return new InstantCommand(() -> gateServo.setPosition(GATE_OPEN_POSITION));
    }

    public Command closeGate() {
        return new InstantCommand(() -> gateServo.setPosition(GATE_CLOSED_POSITION));
    }

    public void update(double rpm, double hoodAngle) {
        targetRPM = rpm;

        double angleDegrees = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, hoodAngle));

        double servoPosition = HOOD_MAX_POSITION -
            (angleDegrees - HOOD_MIN_ANGLE) /
                (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE) *
                (HOOD_MAX_POSITION - HOOD_MIN_POSITION);

        hoodServo.setPosition(servoPosition);
    }

    public boolean atTargetRPM() {
        return Math.abs(getCurrentRPM() - targetRPM) < RPM_TOLERANCE;
    }

    private double getCurrentRPM() {
        return -rightShooterMotor.getVelocity() * 60 / TICKS_PER_REVOLUTION;
    }

    @Override
    public void periodic() {
        double currentRPM = getCurrentRPM();

        if (state.equals(ShooterState.SHOOTING)) {
            double calculatedPower = shooterPIDF.calculate(currentRPM, targetRPM);

            leftShooterMotor.setPower(calculatedPower);
            rightShooterMotor.setPower(calculatedPower);
        }
        else {
            shooterPIDF.reset();

            leftShooterMotor.setPower(0.0);
            rightShooterMotor.setPower(0.0);
        }

        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
    }
}
