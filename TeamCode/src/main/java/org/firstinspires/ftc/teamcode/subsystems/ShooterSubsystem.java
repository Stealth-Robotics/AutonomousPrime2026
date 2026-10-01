package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class ShooterSubsystem extends StealthSubsystem {
    private Servo gateServo;
    private DcMotorEx flywheelMotor;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
    }
}
