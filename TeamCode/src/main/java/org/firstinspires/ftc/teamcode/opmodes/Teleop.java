package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.controls.ControlPanel;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class Teleop extends StealthOpMode {

    private RobotSystem robotSystem;
    private final ControlPanel controls = new ControlPanel(gamepad1, gamepad2);

    @Override
    public void initialize() {
        robotSystem = new RobotSystem(hardwareMap);
        robotSystem.fieldCentricCommand(controls.drive_x(), controls.drive_y(), controls.drive_spin());
    }
}