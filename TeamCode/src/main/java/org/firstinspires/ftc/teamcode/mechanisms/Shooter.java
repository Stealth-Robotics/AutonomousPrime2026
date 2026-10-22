package org.firstinspires.ftc.teamcode.mechanisms;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;

public class Shooter extends StealthSubsystem {
//    private final NextMotor leftShooterMotor;
//    private final NextMotor rightShooterMotor;

    private final NextServo gateServo;

    private GateState gateState = GateState.CLOSED;

    private final double GATE_OPEN_POSITION = 0.18;
    private final double GATE_CLOSED_POSITION = 0.4;

    public enum GateState {
        OPEN,
        CLOSED
    }

    public Shooter(HardwareMap h) {
//        leftShooterMotor = new NextMotor(RobotController.controlHub(), 2);
//        rightShooterMotor = new NextMotor(RobotController.controlHub(), 3);

        gateServo = new NextServo(RobotController.expansionHub(), 0);

//        leftShooterMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
//        leftShooterMotor.setDirection(NextMotor.Direction.FORWARD);
//
//        leftShooterMotor.getVelocityConstants()
//            .withP(0)
//            .withI(0)
//            .withD(0)
//            .withS(0)
//            .withV(0)
//            .withA(0);
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
        telemetry.addData("Gate State", gateState);
    }
}
