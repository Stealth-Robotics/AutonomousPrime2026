package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class RobotSystem extends StealthSubsystem {
    private Follower follower;
    private DrivePowers powers;
    public RobotSystem(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void drive(double x, double y, double rot) {
        powers = ManualDrive.fieldCentric(x, y, rot, follower.pose().heading());
    }

    public Command fieldCentricCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return run(() -> drive(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble()));
    }

    @Override
    public void periodic() {
        follower.manual(powers);
        follower.update();
    }
}
