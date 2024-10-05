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

    private HolonomicOdometry m_odometry;
    private Pose2d m_robotPose;     // absolute coordinates and rotation
    private Motor.Encoder m_left;
    private Motor.Encoder m_right;
    private Motor.Encoder m_perpendicular;
    private double m_initialangledegrees;
    private double encoderRadiusCentimeters = 2.4;
    private double ticksPerRevolution = 2000.0;

    protected PIDController m_translationXController = new PIDController(0.0, 0.0, 0.0);
    protected PIDController m_translationYController = new PIDController(0.0, 0.0, 0.0);
    protected PIDController m_rotationController = new PIDController(0.0, 0.0, 0.0);

    public static double TrackWidthCentimeters = 36;  // distance between odo pods
    public static double DeadWheelRadiusCentimeters = 2.4;
    public static double DeadWheelEncoderRes = 2000.0; // resolution of deadwheels
    public static double PerpendicularOffsetCentimeters = -20.32; // distance of third pod from center of bot

    public static PIDCoefficients TranslationX = new PIDCoefficients(0,0,0);     // D should always be zero
    public static PIDCoefficients TranslationY = new PIDCoefficients(0,0,0);
    public static PIDCoefficients TranslationRotation = new PIDCoefficients(0,0,0);


    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);
        double cm_per_tick = 2 * Math.PI * encoderRadiusCentimeters / ticksPerRevolution;

        m_left = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);         // encoders
        m_left.setDirection(MotorEx.Direction.REVERSE);
        m_right = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);
        m_perpendicular = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);
        m_perpendicular.setDirection(MotorEx.Direction.REVERSE);


        m_odometry = new HolonomicOdometry(
                m_left::getDistance,        // calls functions to get distance
                m_right::getDistance,
                m_perpendicular::getDistance,
                TrackWidthCentimeters,
                PerpendicularOffsetCentimeters
        );

        m_odometry.updatePose(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(0)));
        // gets initial X and Y coordinates passed into constructor, but sets rotation to zero.
        m_robotPose = initialPose;
        m_initialangledegrees = initialPose.getRotation().getDegrees();
    }

    @Override
    public Rotation2d getHeading() {
        return m_odometry.getPose().getRotation().times(-1); // Converting clockwise rotation to counterclockwise
    }

    @Override
    public Pose2d getPose() {
        return m_robotPose;
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose; // resets pose to whatever's passed into function
    }
}
