package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "classAuto", group = "Autos")
public class classAuto extends LinearOpMode {
    Robot mickey = new Robot();
    public enum MoveForward {
        STATE_IDLE,
        STATE_MOVE
    }
    public enum TurnMotors {
        STATE_IDLE,
        STATE_TURN
    }
    MoveForward moveForward = MoveForward.STATE_MOVE;
    TurnMotors turnMotors = TurnMotors.STATE_IDLE;
    @Override
    public void runOpMode() {
        while(opModeInInit()) {
            mickey.initHardware(hardwareMap);
        }
        while (opModeIsActive()) {
            switch (moveForward) {
                case STATE_MOVE: {
                    forward();
                    if (updateDistance() < 5) {
                        stopMotors();
                        moveForward = MoveForward.STATE_IDLE;
                        turnMotors = TurnMotors.STATE_TURN;
                    }
                break; }
                case STATE_IDLE: {

                    break;
                }
            }
            switch (turnMotors) {
                case STATE_IDLE: {
                    break;
                }
                case STATE_TURN:{
                    turnLeft();
                    if (mickey.getIMUHeading() < -90) {
                        mickey.resetIMU();
                        stopMotors();
                        turnMotors = TurnMotors.STATE_IDLE;
                    }
                }
            }
        }

    }
    public double updateDistance() {
        return mickey.distanceSensor.getDistance(DistanceUnit.CM);
    }
    public void forward() {
        mickey.frontleftdrive.setPower(0.5);
        mickey.frontrightdrive.setPower(0.5);
        mickey.backleftdrive.setPower(0.5);
        mickey.backrightdrive.setPower(0.5);
    }
    public void stopMotors() {
        mickey.frontleftdrive.setPower(0.0);
        mickey.frontrightdrive.setPower(0.0);
        mickey.backleftdrive.setPower(0.0);
        mickey.backrightdrive.setPower(0.0);
    }
    public void turnLeft() {
        mickey.frontleftdrive.setPower(0.5);
        mickey.backleftdrive.setPower(0.5);
        mickey.frontrightdrive.setPower(-0.5);
        mickey.backrightdrive.setPower(-0.5);
    }

}
