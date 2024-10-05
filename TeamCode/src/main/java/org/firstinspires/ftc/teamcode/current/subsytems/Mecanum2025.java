package org.firstinspires.ftc.teamcode.current.subsytems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;

public class Mecanum2025 extends BaseMecanumDrive {

    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);


    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(0);
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(0,0,Rotation2d.fromDegrees(0));
    }

    @Override
    public void resetPose(Pose2d pose) {

    }
}
