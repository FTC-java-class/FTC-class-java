package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp", group = "!Teleop")
    public class Teleop3 extends OpMode {
        private DcMotor leftMotor = null;
        private Servo intakeServo = null;
        @Override
        public void init() {
            leftMotor  = hardwareMap.get(DcMotor.class, "left_front_drive");
            intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        }
        @Override
        public void loop() {

        }
    }

