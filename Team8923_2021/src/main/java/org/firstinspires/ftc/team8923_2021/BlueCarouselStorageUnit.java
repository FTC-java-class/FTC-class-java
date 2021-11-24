package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueCarouselStorageUnit")
public class BlueCarouselStorageUnit extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;
            moveForward(-2.8, 10, 10);
            spinCarouselBlue();

            imuPivot(referenceAngle, -90, 35, 0.015, 3.9);
            moveForward(30, 10, 10);
            moveForward(10, 10, 10);
            break;
        }
    }
}