package org.firstinspires.ftc.teamcode.current.testing;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArmMotorTest extends OpMode {
    private MotorEx ArmMotor;

    @Override
    public void init() {
        ArmMotor = new MotorEx(hardwareMap, "arm");
        ArmMotor.set(0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            ArmMotor.set(0.5);
        }
        else if (gamepad1.dpad_down){
            ArmMotor.set(-0.5);
        }
        else {
            ArmMotor.set(0);
        }
    }
}
