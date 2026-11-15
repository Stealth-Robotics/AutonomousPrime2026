package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import java.util.function.BooleanSupplier;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo harvesterServo;

    private final DistanceSensor intakeSensor;

    private static final double HARVESTER_UP_POSITION = 0;
    private static final double HARVESTER_DOWN_POSITION = 0.92;

    private static final double DETECT_DISTANCE = 2.0;
    private static final double CLEAR_DISTANCE = 2.5;

    private static final long FULL_DETECTION_TIME_MS = 250;
    private static final long CLEAR_DETECTION_TIME_MS = 200;

    private boolean isFull = false;

    private long detectionStart = 0;
    private long clearStart = 0;

    private HarvesterState harvesterState = HarvesterState.UP;

    public enum HarvesterState {
        UP,
        DOWN
    }

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intake");
        harvesterServo = h.get(Servo.class, "harvester");
        intakeSensor = h.get(DistanceSensor.class, "intake_sensor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Command teleopIntake(GamepadEx gamepad) {
        return infinite(() -> {
            double power = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (Math.abs(power) > 0.2)
                intakeMotor.setPower(power);
            else
                intakeMotor.setPower(0);
        });
    }

    public Command intake() {
        return instant(() -> intakeMotor.setPower(1));
    }

    public Command outtake() {
        return instant(() -> intakeMotor.setPower(-1));
    }

    public Command stop() {
        return instant(() -> intakeMotor.setPower(0));
    }

    public Command raiseHarvester() {
        return new InstantCommand(() -> {
            harvesterServo.setPosition(HARVESTER_UP_POSITION);
            harvesterState = HarvesterState.UP;
        });
    }

    public Command lowerHarvester() {
        return new InstantCommand(() -> {
            harvesterServo.setPosition(HARVESTER_DOWN_POSITION);
            harvesterState = HarvesterState.DOWN;
        });
    }

    public Command toggleHarvester() {
        return conditional(
            () -> harvesterState.equals(HarvesterState.DOWN),
            raiseHarvester(),
            lowerHarvester()
        );
    }

    private void updateSensor() {
        double distance = intakeSensor.getDistance(DistanceUnit.INCH);
        long now = System.currentTimeMillis();

        if (!Double.isFinite(distance))
            return;

        if (!isFull) {
            if (distance < DETECT_DISTANCE) {
                clearStart = 0;

                if (detectionStart == 0)
                    detectionStart = now;

                if (now - detectionStart >= FULL_DETECTION_TIME_MS)
                    isFull = true;
            }
            else detectionStart = 0;
        }
        else {
            if (distance > CLEAR_DISTANCE) {
                if (clearStart == 0)
                    clearStart = now;

                if (now - clearStart >= CLEAR_DETECTION_TIME_MS) {
                    isFull = false;
                    detectionStart = 0;
                    clearStart = 0;
                }
            }
            else clearStart = 0;
        }
    }

    public boolean isFull() {
        return isFull;
    }

    @Override
    public void periodic() {
        updateSensor();

        telemetry.addData("Is Full", isFull());
        telemetry.addData("Sensor Distance (in)", intakeSensor.getDistance(DistanceUnit.INCH));
    }
}
