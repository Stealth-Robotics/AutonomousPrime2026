package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;
import static org.stealthrobotics.library.Commands.waitUntil;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
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
        return sequence(
            shooter.runShooter(shotCalculator::getRPM),
            waitUntil(shooter::atTargetRPM),
            shooter.openGate(),
            intake.intake(),
            waitSeconds(1000),
            shooter.closeGate(),
            intake.stop(),
            shooter.stopShooter()
        );
    }

    @Override
    public void periodic() {
//        shotCalculator.update(drive.getPose(),[VELOCITY],[TARGET]);
        turret.updateTurretAngle(shotCalculator.getTurretAngle());
        shooter.updateHoodAngle(shotCalculator.getHoodAngle());

        telemetry.addData("Alliance", Alliance.get());
    }
}
