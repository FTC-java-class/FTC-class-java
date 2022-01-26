package org.firstinspires.ftc.team6220_2021;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

import java.util.List;

@Autonomous(name = "SimpleRedRightAuto", group = "Competition")
public class SimpleRedRightAuto extends MasterAutonomous {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        initVuforia();
        initTfod();

        int barcode = 2;

        servoGrabber.setPosition(Constants.CLOSED_GRABBER_POSITION);
        servoArm.setPosition(0.9);

        waitForStart();

        motorArm.setPower(0.5);
        motorArm.setTargetPosition(800);
        servoArm.setPosition(0.667);

        pauseMillis(1000);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 2000) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel().equals("TSE")) {
                        double TSELocation = (recognition.getLeft() + recognition.getRight()) / 2.0;

                        if (TSELocation > 0.0 && TSELocation <= 267.0) {
                            barcode = 0;
                        } else if (TSELocation > 267.0 && TSELocation <= 533.0) {
                            barcode = 1;
                        } else if (TSELocation > 533.0 && TSELocation <= 800.0) {
                            barcode = 2;
                        }
                    }
                }
                telemetry.update();
            }
        }

        telemetry.addData("barcode: ", barcode);
        telemetry.update();

        driveInches(6, Constants.MIN_DRIVE_PWR, true);
        pauseMillis(125);

        switch (barcode) {
            case 0:
                motorArm.setTargetPosition(300);
                servoArm.setPosition(0.55);
                break;
            case 1:
                motorArm.setTargetPosition(550);
                servoArm.setPosition(0.45);
                break;
            case 2:
                motorArm.setTargetPosition(900);
                servoArm.setPosition(0.3);
                break;
        }

        turnToAngle(25);
        pauseMillis(125);
        driveInches(42 / Math.sqrt(3), Constants.MIN_DRIVE_PWR, true);
        pauseMillis(125);
        servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
        pauseMillis(500);

        switch (barcode) {
            case 0:
                driveInches(26, Constants.MIN_DRIVE_PWR, false);
                break;
            case 1:
                driveInches(29, Constants.MIN_DRIVE_PWR, false);
                break;
            case 2:
                driveInches(32, Constants.MIN_DRIVE_PWR, false);
                break;
        }

        pauseMillis(125);
        turnToAngle(-80);
        pauseMillis(125);
        driveInches(30, 0.8, true);

        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
        motorArm.setTargetPosition(0);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TENSORFLOW_MODEL_ASSET, Constants.TENSORFLOW_LABELS);
    }
}