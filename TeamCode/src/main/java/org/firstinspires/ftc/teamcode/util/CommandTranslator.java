package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.ivy.Command;

public class CommandTranslator {
    public static com.arcrobotics.ftclib.command.Command asCommand(Command c) {
        com.arcrobotics.ftclib.command.Command newCommand = new RunCommand(c::execute);
        newCommand.interruptOn(c::done);
        return newCommand;
    }
}
