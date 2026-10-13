package org.firstinspires.ftc.teamcode.systems;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class Intake extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo rampServo;
    private double harvesterPosition;
    public final double HARVESTER_CLOSED = 0.0;
    public final double HARVESTER_OPEN = 1.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        rampServo = hardwareMap.get(Servo.class, "rampServo");

        //Intake motor configuration
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Tune
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setHarvester(double position) {
        rampServo.setPosition(position);
        harvesterPosition = position;
    }

    public Command toggleHarvester() {
        return conditional(() -> (harvesterPosition == HARVESTER_OPEN),
                instant(() -> setHarvester(HARVESTER_CLOSED)),
                instant(() -> setHarvester(HARVESTER_OPEN)));
    }
}
