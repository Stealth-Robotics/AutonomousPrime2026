package org.firstinspires.ftc.teamcode;

import org.stealthrobotics.library.Alliance;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

public class Teleop extends NextOpMode {
    private final Robot robot;

    private final CommandGamepad driverGamepad = new CommandGamepad(gamepad1);
    private final CommandGamepad operatorGamepad = new CommandGamepad(gamepad2);

    public Teleop(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.drive.teleopDrive(driverGamepad).schedule();
        configureBindings();
    }

    @Override
    public void periodic() {
        telemetry.addData("o", "ep");
        Telemetry.log("yo)");
    }

    @Override
    public void end() {
    }

    private void configureBindings() {
        driverGamepad.a().onTrue(robot.resetPose());
        driverGamepad.y().onTrue(robot.shooter.toggleGate());
        driverGamepad.x().onTrue(robot.intake.toggleHarvester());
        driverGamepad.rightTrigger().isOver(0.2).whileTrue(robot.intake.intake());
        driverGamepad.leftTrigger().isOver(0.2).whileTrue(robot.intake.outtake());

        operatorGamepad.y().onTrue(robot.toggleTargetHiveSide());
    }

    @NextTeleop(name = "Blue Teleop")
    public class BlueTeleop extends Teleop { public BlueTeleop(Robot robot) { super(robot); Alliance.set(Alliance.BLUE); } }

    @NextTeleop(name = "Red Teleop")
    public class RedTeleop extends Teleop { public RedTeleop(Robot robot) { super(robot); Alliance.set(Alliance.RED); } }
}