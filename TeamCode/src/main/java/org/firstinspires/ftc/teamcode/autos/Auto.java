package org.firstinspires.ftc.teamcode.autos;

import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Auto extends StealthOpMode {
    protected final AutoBuilder autoBuilder;
    public Auto() {
        this.autoBuilder = new AutoBuilder(hardwareMap);
    }

    @Override
    public void initialize() {
        autoBuilder.initialize();
    }

    @Override
    public void end() {
        autoBuilder.end();
    }
}
