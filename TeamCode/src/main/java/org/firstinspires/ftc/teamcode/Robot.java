package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {
    public DcMotor frontleftdrive = null;
    public DcMotor frontrightdrive = null;
    public DcMotor backleftdrive = null;
    public DcMotor backrightdrive = null;
    public DistanceSensor distanceSensor = null;
    public IMU imu;
    public void initHardware(HardwareMap hardwareMap) {
        frontleftdrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightdrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftdrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightdrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT

                )
        );
        imu.initialize(myIMUparameters);
        resetIMU();
    }
    public void resetIMU() {imu.resetYaw();}
    public double getIMUHeading() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }
}
