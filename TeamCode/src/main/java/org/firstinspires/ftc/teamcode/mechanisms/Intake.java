package org.firstinspires.ftc.teamcode.mechanisms;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final NextServo harvesterServo;

    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -1;

    //TODO: Tune
    private final double HARVESTER_UP_POSITION = 0.0;
    private final double HARVESTER_DOWN_POSITION = 1.0;

    private HarvesterState harvesterState = HarvesterState.UP;

    public enum HarvesterState {
        UP,
        DOWN
    }

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intake");
        harvesterServo = new NextServo(RobotController.expansionHub(), 2);

        intakeMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(NextMotor.Direction.REVERSE);
    }

    public Command intake() {
        return instant(() -> intakeMotor.setThrottle(INTAKE_SPEED));
    }

    public Command outtake() {
        return instant(() -> intakeMotor.setThrottle(OUTTAKE_SPEED));
    }

    public Command stop() {
        return instant(() -> intakeMotor.setThrottle(0));
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
