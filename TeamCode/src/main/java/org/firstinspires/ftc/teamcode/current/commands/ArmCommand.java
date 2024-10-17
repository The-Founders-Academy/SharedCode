package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;

public class ArmCommand extends CommandBase {
    public enum ArmPosition {
        ARM_COLLAPSED_INTO_ROBOT,
        ARM_COLLECT,
        ARM_CLEAR_BARRIER,
        ARM_SCORE_SPECIMEN,
        ARM_SCORE_SAMPLE_IN_LOW,
        ARM_ATTACH_HANGING_HOOK,
        ARM_WINCH_ROBOT
    }

    private Arm2025 m_arm;
    private ArmPosition m_armPosition;

    public ArmCommand(Arm2025 armSubsystem, ArmPosition armPosition) {
        m_armPosition = armPosition;
        m_arm = armSubsystem;

    }

    public void initialize() {
        switch (m_armPosition) {
            case ARM_COLLAPSED_INTO_ROBOT:
                m_arm.setArmPosition(m_arm.getARM_COLLAPSED_INTO_ROBOT());
            case ARM_COLLECT:
                m_arm.setArmPosition(m_arm.getARM_COLLECT());






        }

    }
}
