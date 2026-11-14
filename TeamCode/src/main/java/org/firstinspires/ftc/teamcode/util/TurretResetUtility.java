package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Utility(name = "Turret Reset")
public class TurretResetUtility extends StealthOpMode {
    private DcMotorEx turretMotor;

    @Override
    public void initialize() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        //Reset the internal encoder to 0 ticks
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        schedule(new RunCommand(() ->
            telemetry.addData("Turret Position (ticks)", turretMotor.getCurrentPosition())));
    }
}
