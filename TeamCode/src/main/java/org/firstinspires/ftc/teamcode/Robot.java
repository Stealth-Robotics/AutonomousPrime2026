package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;
import static org.stealthrobotics.library.Commands.waitUntil;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.ShotValues;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private final ShotCalculator shotCalculator;
    private final LoopTimeTracker loopTimeTracker;

    // The time it takes for all 4 pollen to leave the shooter
    private static final int SHOT_TIME = 800;

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);

        shotCalculator = new ShotCalculator();
        loopTimeTracker = new LoopTimeTracker();
    }

    public void changeRPMOffset(int amount) {
        shotCalculator.rpmOffset += amount;
    }

    public Command shoot() {
        return sequence(
            shooter.runShooter(),
            waitUntil(shooter::atTargetRPM).withTimeout(1500),
            waitUntil(() -> !turret.isWrapping()).withTimeout(1000),
            shooter.openGate(),
            intake.intake(),
            waitSeconds(SHOT_TIME),
            shooter.closeGate(),
            intake.stop(),
            shooter.stopShooter()
        );
    }

    @Override
    public void periodic() {
        Pose robotPose = drive.getPose();
        Velocity robotVelocity = drive.getVelocity();

        shotCalculator.update(Pose3D.of(robotPose), robotVelocity);
        ShotValues shotValues = shotCalculator.getLatestShotValues();

        turret.update(shotValues.turretAngle, shotValues.turretVelocity);
        shooter.update(shotValues.rpm, shotValues.hoodAngle);

        loopTimeTracker.update();

        telemetry.addData(
            "Pose", "(%.2f in, %.2f in, %.2f deg)",
            robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())
        );

        telemetry.addData(
            "Velocity", "(%.2f in/s, %.2f in/s, %.2f deg/s)",
            robotVelocity.vx, robotVelocity.vy, Math.toDegrees(robotVelocity.omega)
        );
    }
}
