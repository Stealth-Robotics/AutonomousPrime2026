package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    private Robot robot;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Schedule the teleop driving command
        robot.drive.teleopDrive(driverGamepad).schedule();

        // Go to the correct starting configuration
        robot.intake.raiseHarvester().schedule();
        robot.shooter.closeGate().schedule();

        configureBindings();
    }

    @Override
    public void onStart() {
        // Load the saved pose
        robot.drive.setPose(PoseSaver.savedPose);
    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    private void configureBindings() {
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.intake.toggleHarvester());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.shoot());

        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
            .whenActive(robot.intake.intake())
            .whenInactive(robot.intake.stop());
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2)
            .whenActive(robot.intake.outtake())
            .whenInactive(robot.intake.stop());

        // Relocalize the robot to one of the field's corners (assumes that the robot is facing forward)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.FAR_LEFT_CORNER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.FAR_RIGHT_CORNER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.CLOSE_RIGHT_CORNER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.CLOSE_LEFT_CORNER));
    }

    @TeleOp(name = "Teleop (Blue)", group = "Blue")
    public static class BlueTeleop extends Teleop { }

    @TeleOp(name = "Teleop (Red)", group = "Red")
    public static class RedTeleop extends Teleop { }
}