package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor intakeMotor = new NextMotor(RobotController.controlHub(), HardwareConfig.INTAKE_MOTOR);
    private final NextServo harvesterServo = new NextServo(RobotController.controlHub(), HardwareConfig.HARVESTER_SERVO);

    //TODO: Tune
    private final double INTAKE_SPEED = 1.0;
    private final double OUTTAKE_SPEED = -1.0;

    //TODO: Tune
    private final double HARVESTER_UP_POSITION = 0.0;
    private final double HARVESTER_DOWN_POSITION = 1.0;

    private HarvesterState harvesterState = HarvesterState.UP;

    public enum HarvesterState {
        UP,
        DOWN
    }

    public Intake() {
        intakeMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(NextMotor.Direction.FORWARD);

        //Start with the harvester up
        harvesterServo.setPosition(HARVESTER_UP_POSITION);
    }

    public Command intake() {
        return infinite(() -> intakeMotor.setThrottle(INTAKE_SPEED))
            .setEnd(interrupted -> intakeMotor.setThrottle(0));
    }

    public Command outtake() {
        return infinite(() -> intakeMotor.setThrottle(OUTTAKE_SPEED))
            .setEnd(interrupted -> intakeMotor.setThrottle(0));
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
}
