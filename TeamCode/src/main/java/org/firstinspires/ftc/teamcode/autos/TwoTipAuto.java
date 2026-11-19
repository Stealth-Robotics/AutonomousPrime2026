package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class TwoTipAuto extends Auto {
    @Override
    public void buildPaths() {


//        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            robot.shoot()
            //then go get garden and go to other side of field and shoot
            //then grab near flower and shoot that
            //then go back to original flower and grab that and shoot it
            //then go park
        );
    }

    @Autonomous(name = "Two Tip Auto (Blue)", group = "Blue")
    public static class BlueAuto extends TwoTipAuto { }

    @Autonomous(name = "Two Tip Auto (Red)", group = "Red")
    public static class RedAuto extends TwoTipAuto { }
}
