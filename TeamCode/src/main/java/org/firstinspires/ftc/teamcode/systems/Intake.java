package org.firstinspires.ftc.teamcode.systems;

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

import org.stealthrobotics.library.StealthSubsystem;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo harvesterServo;

    private static final double HARVESTER_UP_POSITION = 0.1;
    private static final double HARVESTER_DOWN_POSITION = 0.92;

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
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Command teleopIntakeDefaultCommand(GamepadEx gamepad) {
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
}
