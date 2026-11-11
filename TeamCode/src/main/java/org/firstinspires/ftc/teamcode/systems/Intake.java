package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.BooleanSupplier;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo harvesterServo;

    //Tunable to control bps while shooting
    private static final double TRANSFER_SPEED = 1;

    private static final double HARVESTER_UP_POSITION = 0.15;
    private static final double HARVESTER_DOWN_POSITION = 0.93;

    private HarvesterState harvesterState = HarvesterState.UP;

    public enum HarvesterState {
        UP,
        DOWN
    }

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intake");
        harvesterServo = h.get(Servo.class, "harvester");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Command teleopIntake(GamepadEx gamepad) {
        return infinite(() -> {
            double power = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (Math.abs(power) > 0.2)
                intakeMotor.setPower(power);
        });
    }

    public Command intake() {
        return new InstantCommand(() -> intakeMotor.setPower(1));
    }

    public Command transfer() {
        return new InstantCommand(() -> intakeMotor.setPower(TRANSFER_SPEED));
    }

    public Command outtake() {
        return new InstantCommand(() -> intakeMotor.setPower(-1));
    }

    public Command stop() {
        return new InstantCommand(() -> intakeMotor.setPower(0));
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
}
