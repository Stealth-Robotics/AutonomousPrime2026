package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    @Override
    public void periodic() {
    }
}
