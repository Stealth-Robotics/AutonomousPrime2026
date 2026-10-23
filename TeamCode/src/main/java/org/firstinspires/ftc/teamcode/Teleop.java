package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        // Load the saved pose
        robot.drive.setPose(PoseSaver.savedPose);

        // Schedule the teleop driving command
        robot.drive.teleopDrive(driverGamepad).schedule();

        // Go to the correct starting configuration
        robot.shooter.closeGate().schedule();
        robot.intake.raiseHarvester().schedule();

        configureBindings();
    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    private void configureBindings() {
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(robot.resetPose());
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(robot.shooter.toggleGate());
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.intake.toggleHarvester());

        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
            .whenActive(robot.intake.intake())
            .whenInactive(robot.intake.stop());
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2)
            .whenActive(robot.intake.outtake())
            .whenInactive(robot.intake.stop());
    }

    @TeleOp(name = "Teleop (Blue)", group = "Blue")
    public static class BlueTeleop extends Teleop {
        public BlueTeleop() {
            Alliance.set(Alliance.BLUE);
        }
    }

    @TeleOp(name = "Teleop (Red)", group = "Red")
    public static class RedTeleop extends Teleop {
        public RedTeleop() {
            Alliance.set(Alliance.RED);
        }
    }
}