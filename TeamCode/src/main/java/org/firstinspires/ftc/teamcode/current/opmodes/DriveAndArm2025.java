package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.current.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.current.commands.WristCommand;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;

import java.util.Collections;
import java.util.Set;

@TeleOp()
public class

DriveAndArm2025 extends CommandOpMode {
    private Mecanum2025 m_mecanumDrive;
    private CommandGamepad m_driver;
    private Arm2025 armSubsystem;

    public String NotPressed;

    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(MotorEx.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2025(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), BaseMecanumDrive.Alliance.RED);
        m_driver = new CommandGamepad(gamepad1, 0.2, 0.2);
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));

    }

    public void execute() {
        if(gamepad1.a) {
            Command folded_in = new WristCommand(armSubsystem, WristCommand.WristPosition.FOLDED_IN);
            schedule(folded_in);

            NotPressed = "Pressed";
        }
        else {
            NotPressed = "No longer Pressed";
        }
    }
}