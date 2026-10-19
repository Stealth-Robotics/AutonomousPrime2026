package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PedroPoseSaver;
import org.stealthrobotics.library.Alliance;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

public class ExampleAuto extends NextOpMode {
    private final Robot robot;

    public ExampleAuto(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Alliance.set(alliance);

        //Flip poses based on alliance here
    }

    @Override
    public void start() {
        //Run the auto here
    }

    @Override
    public void end() {
        //Save the end of auto pose
        PedroPoseSaver.savedPose = robot.drive.getPose();
    }

    @NextAutonomous(name = "Blue Example Auto", group = "NextFTC Auto", preselectTeleop = "Blue Teleop")
    public static class BlueAuto extends ExampleAuto {
        public BlueAuto(Robot robot) {
            super(robot, Alliance.BLUE);
        }
    }

    @NextAutonomous(name = "Red Example Auto", group = "NextFTC Auto", preselectTeleop = "Red Teleop")
    public static class RedAuto extends ExampleAuto {
        public RedAuto(Robot robot) {
            super(robot, Alliance.RED);
        }
    }
}
