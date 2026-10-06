package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo rampServo;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        rampServo = hardwareMap.get(Servo.class, "rampServo");

        //Intake motor configuration
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Tune
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
