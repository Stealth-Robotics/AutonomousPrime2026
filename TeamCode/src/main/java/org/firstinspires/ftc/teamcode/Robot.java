package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;
import static org.stealthrobotics.library.Commands.waitUntil;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.ShotTimer;
import org.firstinspires.ftc.teamcode.util.ShotValues;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private final ShotCalculator shotCalculator;
    private final LoopTimeTracker loopTimeTracker;

    private final ShotTimer shotTimer;

    private boolean resumeShooting = false;

    // The time it takes for all 4 pollen to leave the shooter
    private static final double SHOT_TIME_SECONDS = 0.6;

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);

        shotCalculator = new ShotCalculator();
        loopTimeTracker = new LoopTimeTracker();

        shotTimer = new ShotTimer();
    }

    public Command shoot() {
        return sequence(
            turret.setTracking(),
            shooter.runShooter(),

            new InstantCommand(shotTimer::reset),
            new InstantCommand(shotTimer::resume),
            shooter.openGate(),
            intake.intake(),
            new RunCommand(() -> {
                if (turret.atAngle() && shooter.atTargetRPM() && !turret.isWrapping()) {
                    if (resumeShooting) {
                        shotTimer.resume();

                        shooter.openGate().schedule();
                        intake.intake().schedule();

                        resumeShooting = false;
                    }
                }
                else if (!shotTimer.isPaused()) {
                    shotTimer.pause();

                    shooter.closeGate().schedule();
                    intake.stop().schedule();

                    resumeShooting = true;
                }
            }).interruptOn(() -> shotTimer.seconds() >= SHOT_TIME_SECONDS),
            new InstantCommand(shotTimer::reset),

            shooter.closeGate(),
            intake.stop(),
            shooter.stopShooter(),
            turret.setHomed()
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

        telemetry.addData("Turret Wrapping", turret.isWrapping());
        telemetry.addData("Turret At Angle", turret.atAngle());
        telemetry.addData("At RPMS", shooter.atTargetRPM());

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
