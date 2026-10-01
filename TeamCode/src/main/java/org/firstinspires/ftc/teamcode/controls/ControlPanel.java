package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;

public class ControlPanel {
    private final GamepadEx driver;
    private final GamepadEx operator;

    public ControlPanel(Gamepad driver, Gamepad operator) {
        this.driver = new GamepadEx(driver);
        this.operator = new GamepadEx(operator);
    }

    public DoubleSupplier drive_x() {
        return (() -> -driver.getLeftY());
    }

    public DoubleSupplier drive_y() {
        return (driver::getLeftX);
    }

    public DoubleSupplier drive_spin() {
        return (driver::getRightX);
    }
}

