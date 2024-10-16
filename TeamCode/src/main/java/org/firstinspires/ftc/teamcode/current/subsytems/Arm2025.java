package org.firstinspires.ftc.teamcode.current.subsytems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm2025 {

    public DcMotor armMotor = null; //the arm motor
//    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo

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

    public double armPosition = ARM_COLLAPSED_INTO_ROBOT;


    public Arm2025(DcMotor armMotor, Servo wrist) {
        this.armMotor = armMotor;
        this.wrist = wrist;

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition(double position) {
        armPosition = position;
        armMotor.setTargetPosition((int) armPosition);
        ((DcMotorEx) armMotor).setVelocity(2100);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

//    public void setIntakePower(double power) {
//        intake.set(power);
//    }

    public void adjustArmPositionWithTriggers(double triggerAdjustment) {
        double adjustedPosition = armPosition + (FUDGE_FACTOR * triggerAdjustment);
        armMotor.setTargetPosition((int) adjustedPosition);
    }


    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }
}

