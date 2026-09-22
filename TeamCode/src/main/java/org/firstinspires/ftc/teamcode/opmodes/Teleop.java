package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.stealthrobotics.library.opmodes.StealthOpMode;

@SuppressWarnings("unused")
@TeleOp(name = "Teleop")
public class Teleop extends StealthOpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void initialize() {
    }
}