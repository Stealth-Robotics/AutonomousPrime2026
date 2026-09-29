package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class Teleop extends StealthOpMode {

    private RobotSystem robotSystem;
    private final GamepadEx driverController = new GamepadEx(gamepad1);

    @Override
    public void initialize() {
        robotSystem = new RobotSystem(hardwareMap);
        robotSystem.drive(-driverController.getLeftY(), driverController.getLeftX(), driverController.getRightX());
    }
}