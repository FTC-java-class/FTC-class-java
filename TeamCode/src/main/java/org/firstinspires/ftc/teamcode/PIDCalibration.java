package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "PID Calib")
public class PIDCalibration extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private BNO055IMU imu;
    private Robot_2022FF robot;
    private static double kp, kd;

    public static double getKp(){return kp;}
    public static double getKd(){return kd;}

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new Robot_2022FF(motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft,imu,this);
        robot.setupRobot();
        kp = 1;
        kd = 0;//Todo make note of the numbers you use!!! Copy over to Robot_2022FF when finished!
        waitForStart();
        boolean auto = false;//note: code starts as driver controlled
        int switchInt = 1;
        double cms = 200;//todo adjust cm as needed, longer distance allows for more accuracy if you have room
        while(opModeIsActive()){
            if(auto){
                robot.pidTunerStrafe(0,switchInt*cms);
            }
            else {
                double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI / 4);
                double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
                double rotation = gamepad1.left_stick_x;

                double powerOne = r * Math.cos(angle);
                double powerTwo = r * Math.sin(angle);

                motorFrontLeft.setPower((powerOne - (rotation)));
                motorFrontRight.setPower((powerTwo - (rotation)));
                motorBackLeft.setPower((powerTwo + (rotation)));
                motorBackRight.setPower((powerOne + (rotation)));

            }
            if(gamepad1.left_bumper)//left bumper to switch modes
                auto=!auto;
            if(gamepad1.right_bumper)//right bumper to change directions. In theory, after reaching destination it should stop... todo if it shakes/doesn't stop let me know!
                switchInt = -1*switchInt;
            if(gamepad1.a)//increase kp
                kp++;
            if(gamepad1.b)//decrease kp
                kp--;
            if(gamepad1.x)//increase kd
                kd++;
            if(gamepad1.y)//decrease kd
                kd--;
            //we don't have access to FTC dashboard, so tuning is done here
            //after each run, change kp and kd accordingly(instructions in discord)
            //press right bumper to resume run
            //press left bumper if need to reset robot
            //press right bumper BEFORE left bumper if you're in driver control mode
            //if about to hit something, please HIT STOP don't try left bumper!!!!!!!!
        }
    }
}
