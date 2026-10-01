package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class IntakeSubsystem extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo extractorServo;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        extractorServo = hardwareMap.get(Servo.class, "extractorServo");
    }
}
