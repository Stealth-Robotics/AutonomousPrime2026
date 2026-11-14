package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Robot;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Utility
public class TurretResetUtility extends StealthOpMode {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);

        // Schedule the turret reset command
        robot.turret.resetToZero().schedule();
    }

    @Override
    public void end() {
    }
}
