/*
starting blue carousel
strafe to spin carousel
forward to end of blue block
move to deliver cube
spline to blue square to park storage unit
 */
package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.webcam.MockDetectTSEPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateArm;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateIntake;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateWebCam;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class P1RedPath3 {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand sample1Follower1;
    private TrajectoryFollowerCommand sample1Follower2;
    private TrajectoryFollowerCommand sample1Follower3;
    private TrajectoryFollowerCommand sample1Follower4;

    private FtcDashboard dashboard;

    private SequentialCommandGroup carouselGroupBlue1;

    private Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    public P1RedPath3(HardwareMap hwMap, Telemetry telemetry){
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public P1RedPath3(HardwareMap hwMap, FtcDashboard db, Telemetry telemetry){
        this.hwMap = hwMap;
        dashboard = db;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public void createPath(){
        startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        CreateCarousel createCarousel = new CreateCarousel(hwMap,"carousel",telemetry);
        CreateWebCam createWebCam = new CreateWebCam(hwMap, "Webcam 1", dashboard, telemetry);
        CreateArm createArm = new CreateArm(hwMap, "arm", telemetry);

        createArm.createAuto();

        createWebCam.createAuto();
        WebCamSubsystem webCamSubsystem = createWebCam.getWebCamSubsystem();

        MockDetectTSEPosition mockDetectTSEPosition = createWebCam.getMockDetectTSEPositionCommand();
        mockDetectTSEPosition.schedule();

        createCarousel.createAuto();
        carouselGroupBlue1 = new SequentialCommandGroup(createCarousel.getMoveCarouselToPosition(),
                new WaitUntilCommand(createCarousel.hasMaxEncoderCountSupplier()).andThen(createCarousel.getStopCarousel()));

        CreateIntake createIntake = new CreateIntake(hwMap, "intake", telemetry);
        createIntake.createAuto();


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.strafeTo(new Vector2d(-60, 60))
                .splineToLinearHeading(new Pose2d(-55, -60, Math.toRadians(245)),Math.toRadians(180))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(-55, -24, Math.toRadians(0)),Math.toRadians(90))
                .strafeTo(new Vector2d(-34.58, -24))
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 2", "performing path 2 action");
                    SetArmLevel setArmLevel = createArm.createSetArmLevel(webCamSubsystem.getLevel());
                    setArmLevel.schedule();
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-60, -33, Math.toRadians(90)),Math.toRadians(90))
                .addDisplacementMarker(()->{
                    createIntake.getSeGrabber().schedule();
                    new WaitCommand(800)
                            .andThen(createIntake.getStopIntake()).schedule();
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(-33, -64, Math.toRadians(90)),Math.toRadians(0))
                .strafeTo(new Vector2d(44,-64))
                .strafeTo(new Vector2d(44, -40))
                .build();


        sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);
        sample1Follower3 = new TrajectoryFollowerCommand(drive,traj3);
        sample1Follower4 = new TrajectoryFollowerCommand(drive,traj4);
    }

    public void execute(CommandOpMode commandOpMode){
        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(
                sample1Follower1.andThen(carouselGroupBlue1,sample1Follower2,sample1Follower3, sample1Follower4)
        ));
    }
}
