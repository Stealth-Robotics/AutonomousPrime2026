package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo harvesterServo;

    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = -1;

    private static final double HARVESTER_UP_POSITION = 0.1;
    private static final double HARVESTER_DOWN_POSITION = 1;

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

    public Command intake() {
        return instant(() -> intakeMotor.setPower(INTAKE_SPEED));
    }

    public Command outtake() {
        return instant(() -> intakeMotor.setPower(OUTTAKE_SPEED));
    }

    public Command stop() {
        return instant(() -> intakeMotor.setPower(0));
    }

    public HarvesterState getHarvesterState() {
        return harvesterState;
    }

    public Command raiseHarvester() {
        return instant(() -> {
            harvesterServo.setPosition(HARVESTER_UP_POSITION);
            harvesterState = HarvesterState.UP;
        });
    }

    public Command lowerHarvester() {
        return instant(() -> {
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

    @Override
    public void periodic() {
    }
}
