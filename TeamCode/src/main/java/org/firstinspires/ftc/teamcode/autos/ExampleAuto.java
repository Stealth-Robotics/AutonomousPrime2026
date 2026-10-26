package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class ExampleAuto extends StealthOpMode {
    private final AutoBuilder autoBuilder;

    public ExampleAuto() {
        autoBuilder = new AutoBuilder(hardwareMap);
    }

    @Override
    public void initialize() {
        autoBuilder.initialize();
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
            autoBuilder.followPath(autoBuilder.startToShoot)
        );
    }

    @Override
    public void end() {
        autoBuilder.end();
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
