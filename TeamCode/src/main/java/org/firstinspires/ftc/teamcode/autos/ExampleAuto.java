package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class ExampleAuto extends Auto {
    @Override
    public Command getAutoCommand() {
        return sequence(
            autoBuilder.followPath(autoBuilder.startToShoot),
            autoBuilder.shoot()
        );
    }

    @Autonomous(name = "Example Auto (Blue)", group = "Blue")
    public static class BlueAuto extends ExampleAuto {
        public BlueAuto() {
            Alliance.set(Alliance.BLUE);
        }
    }

    @Autonomous(name = "Example Auto (Red)", group = "Red")
    public static class RedAuto extends ExampleAuto {
        public RedAuto() {
            Alliance.set(Alliance.RED);
        }
    }
}
