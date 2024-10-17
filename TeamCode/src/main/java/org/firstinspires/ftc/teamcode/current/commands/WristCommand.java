package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;


public class WristCommand extends CommandBase {

    public enum WristPosition {
        FOLDED_IN,
        FOLDED_OUT
    }

    private Arm2025 m_arm;
    private WristPosition m_wristPosition;

    public WristCommand(Arm2025 armSubsystem, WristPosition wristPosition) {
        m_wristPosition = wristPosition;
        m_arm = armSubsystem;

    }

    @Override
    public void initialize() {
        switch (m_wristPosition) {
            case FOLDED_IN:
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_IN());
            case FOLDED_OUT:
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_OUT());
        }
    }


}
