package org.firstinspires.ftc.teamcode.systems;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Shooter extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo gateServo;
    public final double GATE_OPEN = 1.0;
    public final double GATE_CLOSED = 0.0;
    private double latchPosition = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        //Shooter motor configuration
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Tune
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setGate(GATE_OPEN);
    }

    public void setGate(double position) {
        gateServo.setPosition(position);
        latchPosition = position;
    }

    public Command openGate() {
        return instant(() -> setGate(GATE_OPEN));
    }

    public Command closeGate() {
        return instant(() -> setGate(GATE_CLOSED));
    }

    public Command toggleGate() {
        return conditional(
                () -> (latchPosition == GATE_CLOSED),
                openGate(),
                closeGate());
    }

    @Override
    public void periodic() {
    }
}
