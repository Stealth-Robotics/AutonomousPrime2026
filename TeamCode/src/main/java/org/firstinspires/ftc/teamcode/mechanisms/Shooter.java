package org.firstinspires.ftc.teamcode.mechanisms;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Shooter extends StealthSubsystem {
//    private final DcMotorEx leftShooterMotor;
//    private final DcMotorEx rightShooterMotor;

    private final Servo gateServo;

    private GateState gateState = GateState.CLOSED;

    private final double GATE_OPEN_POSITION = 0.18;
    private final double GATE_CLOSED_POSITION = 0.4;

    public enum GateState {
        OPEN,
        CLOSED
    }

    public Shooter(HardwareMap h) {
//        leftShooterMotor = h.get(DcMotorEx.class, "left_shooter");
//        rightShooterMotor = h.get(DcMotorEx.class, "right_shooter");

        gateServo = h.get(Servo.class, "gate");

//        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
