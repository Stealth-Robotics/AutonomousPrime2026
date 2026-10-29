package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class PreloadAuto extends Auto {
    @Override
    public Command getAutoCommand() {
        return autoBuilder.shoot();
    }

    @Autonomous(name = "Preload Auto (Blue)", group = "Blue")
    public static class BluePreload extends PreloadAuto {
        public BluePreload() {
            Alliance.set(Alliance.BLUE);
        }
    }

    @Autonomous(name = "Preload Auto (Red)", group = "Red")
    public static class RedPreload extends PreloadAuto {
        public RedPreload() {
            Alliance.set(Alliance.RED);
        }
    }
}

