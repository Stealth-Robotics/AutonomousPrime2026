package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {
    private Robot robot;
    private GamepadEx driverGamepad, operatorGamepad;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Setup default commands
        robot.drive.setDefaultCommand(robot.drive.teleopDrive(driverGamepad));
        robot.intake.setDefaultCommand(robot.intake.teleopIntake(driverGamepad));

        // Go to the correct starting configuration
        robot.intake.raiseHarvester().schedule();
        robot.shooter.closeGate().schedule();

        configureBindings();
    }

    @Override
    public void onStart() {
        robot.drive.setPose(PoseSaver.savedPose);
    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    private void configureBindings() {
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.intake.toggleHarvester());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(robot.shooter.runShooter(), robot.shooter.stopShooter());

        //TODO: Temp controls change for testing
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.shoot());

        // Relocalize the robot to a known field reset pose (assumes that the robot is facing forward)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.LEFT_SIDE_OF_FLOWER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.NEAR_RIGHT_CORNER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.RIGHT_SIDE_OF_FLOWER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.NEAR_LEFT_CORNER));
    }

    @TeleOp(name = "Teleop (Blue)", group = "Blue")
    public static class BlueTeleop extends Teleop { }

    @TeleOp(name = "Teleop (Red)", group = "Red")
    public static class RedTeleop extends Teleop { }
}