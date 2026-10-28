package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;
import static org.stealthrobotics.library.Commands.waitUntil;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.stealthrobotics.library.Alliance;
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
        return sequence(
            shooter.runShooter(),
            waitUntil(shooter::atTargetRPM),
            shooter.openGate(),
            intake.intake(),
            waitSeconds(2000),
            shooter.closeGate(),
            intake.stop(),
            shooter.stopShooter()
        );
    }

    @Override
    public void periodic() {
        shotCalculator.update(Pose3D.of(drive.getPose()), drive.getVelocity());

        turret.update(0, 0);
        shooter.update(3400, 22.5);
//        shooter.update(shotCalculator.getRPM(), shotCalculator.getHoodAngle());

        telemetry.addData("Alliance", Alliance.get());
    }
}
