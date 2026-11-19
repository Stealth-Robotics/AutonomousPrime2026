package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class Shooter extends StealthSubsystem {
    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;

    private final VoltageSensor voltageSensor;

    private final Servo gateServo;
    private final Servo hoodServo;

    private final PIDFController shooterPIDF = new PIDFController(
        0.0015,
        0,
        0.00001,
        0.00019
    );

    private ShooterState state = ShooterState.STOPPED;

    private static final double RPM_TOLERANCE = 50;

    private static final double HOOD_MIN_POSITION = 0.55;
    private static final double HOOD_MAX_POSITION = 0.76;

    //Hood angles (0 degrees being a shot vertically straight up)
    private static final double HOOD_MIN_ANGLE = 3.838;
    private static final double HOOD_MAX_ANGLE = 15.838;

    private static final double GATE_OPEN_POSITION = 0.18;
    private static final double GATE_CLOSED_POSITION = 0.3;

    private static final double TICKS_PER_REVOLUTION = 28;

    private double targetRPM, hoodOffset;

    public enum ShooterState {
        SHOOTING,
        STOPPED
    }

    public Shooter(HardwareMap h) {
        leftShooterMotor = h.get(DcMotorEx.class, "left_shooter");
        rightShooterMotor = h.get(DcMotorEx.class, "right_shooter");

        gateServo = h.get(Servo.class, "gate");
        hoodServo = h.get(Servo.class, "hood");

        voltageSensor = h.voltageSensor.iterator().next();

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo.setDirection(Servo.Direction.REVERSE);
    }

    public Command hoodCompensation() {
        return Commands.sequence(
            new WaitCommand(100),
            new InstantCommand(() -> hoodOffset += 0.02),
            new WaitCommand(100),
            new InstantCommand(() -> hoodOffset += 0.02),
            new WaitCommand(100),
            new InstantCommand(() -> hoodOffset += 0.02),
            new WaitCommand(100)
        ).whenFinished(() -> hoodOffset = 0);
    }

    public Command runShooter() {
        return new InstantCommand(() -> state = ShooterState.SHOOTING);
    }

    public Command stopShooter() {
        return new InstantCommand(() -> state = ShooterState.STOPPED);
    }

    public Command openGate() {
        return new InstantCommand(() -> gateServo.setPosition(GATE_OPEN_POSITION));
    }

    public Command closeGate() {
        return new InstantCommand(() -> gateServo.setPosition(GATE_CLOSED_POSITION));
    }

    public ShooterState getState() {
        return state;
    }

    public void update(double rpm, double hoodAngle) {
        targetRPM = rpm;

        double angle = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, hoodAngle));

        double servoPosition = HOOD_MAX_POSITION -
            (angle - HOOD_MIN_ANGLE) /
            (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE) *
            (HOOD_MAX_POSITION - HOOD_MIN_POSITION);

        hoodServo.setPosition(MathUtils.clamp(servoPosition + hoodOffset, HOOD_MIN_POSITION, HOOD_MAX_POSITION));
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

        if (!state.equals(ShooterState.STOPPED)) {
            double calculatedPower = shooterPIDF.calculate(currentRPM, targetRPM);

            //Normalize the output to 12V even when the battery voltage is higher
            calculatedPower *= 12.0 / voltageSensor.getVoltage();

            leftShooterMotor.setPower(calculatedPower);
            rightShooterMotor.setPower(calculatedPower);
        }
        else {
            shooterPIDF.reset();

            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
    }
}
