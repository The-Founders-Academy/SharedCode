package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class newDriveAndArmCode extends LinearOpMode {
    public MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;


    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;


    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position in later functions*/
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    double liftPosition = ARM_COLLAPSED_INTO_ROBOT;     // initializes the robot arm position to 0

    // variables to set arm to a specific position
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;

    @Override
    public void runOpMode(){
       // Private Variables to control the drivetrain.
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        m_frontLeft = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        m_frontLeft.setRunMode(Motor.RunMode.RawPower);
        m_frontRight.setRunMode(Motor.RunMode.RawPower);
        m_backLeft.setRunMode(Motor.RunMode.RawPower);
        m_backRight.setRunMode(Motor.RunMode.RawPower);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);


        // getting the IMU from hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Change as needed based on the direction the control hub is facing relative to the front of robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));

        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.update();

            // resets field relative drive forward direction to the direction the robot is facing when called.
            if(gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // between -180 and 180

            // Rotate the movement direction counter to the bot's rotation
            // so If bot rotates 45 degrees counterclockwise, field relative right should then be rotated 45 degrees clockwise
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            telemetry.addData("BotHeading: ", botHeading);
            telemetry.addData("Rotation X", rotX);
            telemetry.addData("Rotation Y", rotY);
            telemetry.update();

            rotX = rotX * 1.1; // counteracts imperfect strafing

            // Normalizes all wheel powers to be within the range -1 to 1 by dividing by the largest wheel speed
            double largestSpeed = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


            // same system as Robot Relative drive except rotY and rotX take into account field relative motions.
            double frontLeftPower = (rotY + rotX + rx) / largestSpeed;
            double backLeftPower = (rotY - rotX + rx) / largestSpeed;
            double frontRightPower = (rotY - rotX - rx) / largestSpeed;
            double backRightPower = (rotY + rotX - rx) / largestSpeed;

            m_frontLeft.set(frontLeftPower);
            m_frontRight.set(frontRightPower);
            m_backLeft.set(backLeftPower);
            m_backRight.set(backRightPower);


            // TODO make if statement for different intake collections

            /* Fudge factor for arm position is created here. After moving to a position like 160 degrees, the fudge factor
            allows you to vary that by + or - 15 degrees, by hitting the left or right triggers. So you have some adjustment
            control if you need it. This means at 160 you could be anywhere between 145 and 175 degrees, as neeeded.
             */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            // If statements to set robot arm, wrist, and intake to different configs

            if(gamepad1.a){
                // This is the intake/collection position
                 armPosition = ARM_COLLECT;
                 wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if(gamepad1.b){
                // This is 20 degrees up from the intake position, so we can go over the metal bar on the ground
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if(gamepad1.x){
                // This is the position to score samples in the low basket
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if(gamepad1.dpad_left){
                // This is the position that collapses the robot arm back into the starting config
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if(gamepad1.dpad_right){
                // Correct height to score specimen on the high chamber
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if(gamepad1.dpad_up) {
                // Moves arm up vertically up, first stage of the level 2 hang
                armPosition = ARM_ATTACH_HANGING_HOOK;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if(gamepad1.dpad_down) {
                // Moves arm up vertically up, first stage of the level 2 hang
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            // Add selected target position for arm plus the fudge factor to create the correct arm height and wrist pose
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
