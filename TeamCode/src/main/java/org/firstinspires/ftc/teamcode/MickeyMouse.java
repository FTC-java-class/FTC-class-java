package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.awt.font.NumericShaper;

@TeleOp(name = "MickeyMouse", group = "Teleop")
public class MickeyMouse extends OpMode {

    //Controller buttons
    boolean aPress1;
    boolean bPress1;

    boolean aPress1Down = false;
    boolean bPress1Down = false;

    boolean dpadDown1;
    boolean dpadUp1;
    boolean dpadDown1_down = false;
    boolean dpadup1_down = false;

    //For joystick
    double frontrightcurrent = 0;
    double frontleftcurrent = 0;
    double backrightcurrent = 0;
    double backleftcurrent = 0;
    double driverTurn = 0;
    double gamepadXControl = 0;
    double gamepadYControl = 0;
    double gamepadXCoordinate = 0;
    double gamepadYCoordinate = 0;
    double gamepadDegree = 0;
    //double robotDegree = 0;
    double movementDegree = 0;
    double gamepadHypot = 0;
    double power = 0.2;
    double robotdegree = 0;
    boolean fieldcentric = false;
    Robot mickey = new Robot();
    //
    static long loopstart = System.currentTimeMillis();
    @Override
    public void init() {
         mickey.initHardware(hardwareMap);
    }
    public void loop() {
        LOOPSTART();
        updateMotor();
        updateController();
        telemetry.addData("power", power);
        telemetry.addData("Frame Per Second", 1000/GETLOOPTIME());
        telemetry.update();
    }
    public static void LOOPSTART() {
        loopstart = System.currentTimeMillis();
    }
    public static double GETLOOPTIME() {
        return((double)(System.currentTimeMillis() - loopstart));
    }
    public void resetIMU() {mickey.imu.resetYaw();}
    public double getIMUHeading() {
        YawPitchRollAngles ypr = mickey.imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }
    public void updateController() {
        dpadDown1_down = gamepad1.dpad_down && !dpadDown1;
        dpadDown1 = gamepad1.dpad_down;
        dpadup1_down = gamepad1.dpad_up && !dpadUp1;
        dpadUp1 = gamepad1.dpad_up;

        if (dpadup1_down) {
            power = power+0.2;
        }
        if (dpadDown1_down) {
            power = power-0.2;
        }

    }

    public void updateMotor() {
        robotdegree = getIMUHeading();
        gamepadXCoordinate = gamepad1.left_stick_x;
        gamepadYCoordinate  = gamepad1.left_stick_y;

        //Angle of the robot
        gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYControl), 0, 1);
        gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);

        movementDegree = gamepadDegree - robotdegree;
        gamepadYControl = Math.sin(movementDegree) * gamepadHypot;
        gamepadXControl = Math.cos(movementDegree) * gamepadHypot;

        frontrightcurrent = (power * (driverTurn+ (-gamepadYControl) + gamepadXControl));
        backrightcurrent = (power * (driverTurn+ (-gamepadYControl) - gamepadXControl));
        frontleftcurrent = (power * (driverTurn+ (-gamepadYControl) - gamepadXControl));
        backleftcurrent = (power * (driverTurn+ (-gamepadYControl) + gamepadXControl));
        mickey.frontleftdrive.setPower(frontleftcurrent);
        mickey.frontrightdrive.setPower(frontrightcurrent);
        mickey.backleftdrive.setPower(backleftcurrent);
        mickey.backrightdrive.setPower(backrightcurrent);
        driverTurn = gamepad1.right_stick_x;




    }
}
