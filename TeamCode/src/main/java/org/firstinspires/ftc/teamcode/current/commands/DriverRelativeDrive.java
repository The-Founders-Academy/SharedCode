package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;

public class DriverRelativeDrive extends CommandBase {
    private Mecanum2025 m_mecanum;
    private CommandGamepad m_driver;

    public DriverRelativeDrive(Mecanum2025 mecanum, CommandGamepad driver) {
        m_mecanum = mecanum;
        m_driver = driver;
        addRequirements(m_mecanum, m_driver);
    }

    @Override
    public void execute() {
        m_mecanum.moveFieldRelative(m_driver.getLeftY(), -m_driver.getLeftX(), -m_driver.getRightX());     // rotation inverted because clockwise rotation is negative
    }
}
