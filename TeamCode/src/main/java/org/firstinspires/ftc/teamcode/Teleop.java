package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.Commands.run;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.systems.Robot;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
    private GamepadEx operatorGamepad;

    private Robot robot;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        configureBindings();
    }

    private void configureBindings() {
        robot.drive.setDefaultCommand(run(() -> {
            double scaledPower = driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 0.75 : 1;
            robot.drive.driveFieldCentric(
                    driveGamepad.getLeftX() * scaledPower,
                    driveGamepad.getLeftY() * scaledPower,
                    driveGamepad.getRightX() * scaledPower
            );
        }, robot.drive));

        //Reset the teleop pose if we ever lose our position
        driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(() -> robot.resetPose());

        //Toggle which side of our hive we are aiming at
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> robot.toggleTargetHiveSide());
    }

    @TeleOp(name = "Teleop (RED ALLIANCE)", group = "Red")
    public static class RedTeleop extends Teleop { }

    @TeleOp(name = "Teleop (BLUE ALLIANCE)", group = "Blue")
    public static class BlueTeleop extends Teleop { }
}