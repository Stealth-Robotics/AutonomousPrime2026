package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.Robot;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class SampleAuto extends StealthOpMode {
    private Robot robot;
    public void initialize() {
        robot = new Robot(hardwareMap);
    }
}
