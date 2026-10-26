package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.Targets;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    private final ShotCalculator shotCalculator;

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);
        shotCalculator = new ShotCalculator();
    }

    public Command shoot() {
        return Commands.sequence(
            shooter.runShooter(shotCalculator::getRPM),
            new WaitUntilCommand(shooter::atTargetRPM),
            shooter.openGate(),
            intake.intake(),
            new WaitCommand(1000),
            shooter.closeGate(),
            intake.stop(),
            shooter.stopShooter()
        );
    }

    @Override
    public void periodic() {
        shotCalculator.update(Pose3D.of(drive.getPose()), drive.getVelocity(), Targets.getTarget(drive.getPose()));
        turret.updateTracking(shotCalculator);
        shooter.updateHoodAngle(shotCalculator);

        telemetry.addData("Alliance", Alliance.get());
    }
}
