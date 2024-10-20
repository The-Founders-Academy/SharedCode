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
        ARM_WINCH_ROBOT,

        LEFT_TRIGGER_PRESSED,

        RIGHT_TRIGGER_PRESSED
    }

    private Arm2025 m_arm;
    private ArmPosition m_armPosition;

    public ArmCommand(Arm2025 armSubsystem, ArmPosition armPosition) {
        m_armPosition = armPosition;
        m_arm = armSubsystem;

        addRequirements(m_arm);

    }

    public void initialize() {
        switch (m_armPosition) {
            case ARM_COLLAPSED_INTO_ROBOT:
                m_arm.setArmPosition(m_arm.getARM_COLLAPSED_INTO_ROBOT());
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_IN());
                break;

            case ARM_COLLECT:
                m_arm.setArmPosition(m_arm.getARM_COLLECT());
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_OUT());
                break;

            case ARM_CLEAR_BARRIER:
                m_arm.setArmPosition(m_arm.getARM_CLEAR_BARRIER());
                break;

            case ARM_SCORE_SPECIMEN:
                m_arm.setArmPosition(m_arm.getARM_SCORE_SPECIMEN());
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_IN());
                break;

            case ARM_SCORE_SAMPLE_IN_LOW:
                m_arm.setArmPosition(m_arm.getARM_SCORE_SAMPLE_IN_LOW());
                m_arm.setWristPosition(m_arm.getWRIST_FOLDED_OUT());
                break;

            case ARM_ATTACH_HANGING_HOOK:
                m_arm.setArmPosition(m_arm.getARM_ATTACH_HANGING_HOOK());
                break;

            case ARM_WINCH_ROBOT:
                m_arm.setArmPosition(m_arm.getARM_WINCH_ROBOT());
                break;

            case LEFT_TRIGGER_PRESSED:
                m_arm.setArmPower(-0.1);
                break;

            case RIGHT_TRIGGER_PRESSED:
                m_arm.setArmPower(0.1);
                break;
        }

    }
}
