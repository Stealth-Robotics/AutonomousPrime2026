package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Shooter extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo gateServo;

    public Shooter(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        //Shooter motor configuration
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Tune
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
    }
}
