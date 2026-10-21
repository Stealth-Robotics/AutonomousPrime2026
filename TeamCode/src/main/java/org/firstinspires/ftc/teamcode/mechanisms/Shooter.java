package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    private final NextMotor leftShooterMotor = new NextMotor(RobotController.controlHub(), 0);
    private final NextMotor rightShooterMotor = new NextMotor(RobotController.controlHub(), 0);

    private final NextServo gateServo = new NextServo(RobotController.controlHub(), 0);

    private GateState gateState = GateState.CLOSED;

    //TODO: Tune
    private final double GATE_OPEN_POSITION = 1.0;
    private final double GATE_CLOSED_POSITION = 0.0;

    public enum GateState {
        OPEN,
        CLOSED
    }

    public Shooter() {
        leftShooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);

        leftShooterMotor.setDirection(NextMotor.Direction.FORWARD);
        rightShooterMotor.setDirection(NextMotor.Direction.FORWARD);

        rightShooterMotor.follow(leftShooterMotor);

        //Start with the gate closed
        gateServo.setPosition(GATE_CLOSED_POSITION);
    }

    public GateState getGateState() {
        return gateState;
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

    public Command toggleGate() {
        return conditional(
            () -> gateState.equals(GateState.CLOSED),
            openGate(),
            closeGate()
        );
    }

    @Override
    public void periodic() {
    }
}
