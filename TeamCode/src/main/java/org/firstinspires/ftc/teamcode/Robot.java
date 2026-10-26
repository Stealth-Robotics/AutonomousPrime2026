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

    private static final Pose BLUE_RESET_POSE = new Pose(0, 0, 0);
    private static final Pose RED_RESET_POSE = new Pose(0, 0, 0);

    // Poses named from the drive station point of view, with robot facing towards opposing color's side
    private static final Pose RED_RIGHT_RESET_POSE = new Pose(8,8,0);
    private static final Pose RED_LEFT_RESET_POSE = new Pose(136, 8, 0);
    private static final Pose BLUE_RIGHT_RESET_POSE = new Pose(136,136,180);
    private static final Pose BLUE_LEFT_RESET_POSE = new Pose(8,136,180);

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);
        shotCalculator = new ShotCalculator();
    }

    public Command shoot() {
        return Commands.sequence(
            shooter.runShooter(() -> 3800), //Get from sotm solver
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
//        shotCalculator.update(drive.getPose(),[VELOCITY],[TARGET]);
        turret.updateTracking(shotCalculator);
        shooter.updateHoodAngle(shotCalculator);

        telemetry.addData("Alliance", Alliance.get());
    }
}
