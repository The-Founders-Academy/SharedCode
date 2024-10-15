package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;

public class MoveArm extends CommandBase {
    private Arm2025 armSubsystem;
    private double targetPosition;
    private double wristPosition;
    private double intakePower;

    public void MoveArmCommand(Arm2025 armSubsystem, double targetPosition, double wristPosition, double intakePower) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        this.wristPosition = wristPosition;
        this.intakePower = intakePower;
    }

    @Override
    public void execute() {
        armSubsystem.setArmPosition(targetPosition);
        armSubsystem.setWristPosition(wristPosition);
        armSubsystem.setIntakePower(intakePower);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getArmPosition() - targetPosition) < 5;  // Close to target
    }
}
