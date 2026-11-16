package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;
import static org.stealthrobotics.library.Commands.waitUntil;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.util.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.SimpleShotCalculator;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private final SimpleShotCalculator shotCalculator;
    private final LoopTimeTracker loopTracker;

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);

        shotCalculator = new SimpleShotCalculator();
        loopTracker = new LoopTimeTracker();

        //Trigger that starts idling the shooter when the intake is full
        new Trigger(intake::isFull).whenActive(shooter.idleShooter());
    }

    public Command shoot() {
        return sequence(
            shooter.runShooter(),
            waitUntil(shooter::atTargetRPM).withTimeout(2000),
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
        shotCalculator.update(Pose3D.of(drive.getPose()), drive.getVelocity());

        turret.update(shotCalculator.getTurretAngle());
        shooter.update(shotCalculator.getRPM(), shotCalculator.getHoodAngle());

        loopTracker.update();
    }
}
