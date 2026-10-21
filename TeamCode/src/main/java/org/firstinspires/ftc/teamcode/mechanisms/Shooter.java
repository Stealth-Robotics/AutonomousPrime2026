package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

public class Shooter implements Mechanism {
//    private final NextMotor leftShooterMotor = new NextMotor(RobotController.controlHub(), 1);
//    private final NextMotor rightShooterMotor = new NextMotor(RobotController.controlHub(), 2);

    private final NextServo gateServo = new NextServo(RobotController.controlHub(), 0);

    private GateState gateState = GateState.CLOSED;

    private final double GATE_OPEN_POSITION = 0.18;
    private final double GATE_CLOSED_POSITION = 0.4;

    public enum GateState {
        OPEN,
        CLOSED
    }

    public Shooter() {
        //Uncomment when using
//        leftShooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
//        rightShooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
//
//        leftShooterMotor.setDirection(NextMotor.Direction.FORWARD);
//        rightShooterMotor.setDirection(NextMotor.Direction.FORWARD);
//
//        rightShooterMotor.follow(leftShooterMotor);
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
        Telemetry.log("Gate State", gateState);
    }
}
