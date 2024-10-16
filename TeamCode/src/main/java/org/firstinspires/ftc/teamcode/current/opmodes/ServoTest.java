package org.firstinspires.ftc.teamcode.current.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    public Servo wrist;


    @Override
    public void runOpMode(){

        wrist = hardwareMap.get(Servo.class, "wrist");

        waitForStart();

        while(opModeIsActive()) {

        if(gamepad1.a) {
            wrist.setPosition(0.5);
        }
        else if(gamepad1.b) {
            wrist.setPosition(0.75);
        }

        }
    }
}
