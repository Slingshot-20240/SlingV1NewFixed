package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class SpecNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private static IntakeConstants.ActiveIntakeStates activeIntakeStates;
    private Intake intake;
    private Outtake outtake;

    public static double hpX = 43;
    public static double hpY = -65;
    public static double scoreY = -33;
    public static double scoreGap = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        outtake = robot.outtake;
        //specimenClaw = robot.specimenClaw;
        robot.intake.extendoFullRetract();
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(13.5, -64.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preloaded spec
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slides (small)
                    //flip arm to score
                })
                .lineToConstantHeading(new Vector2d(6,  scoreY))

                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //lower slides (small)
                    //open claw (preloaded spec)
                })

                //pickup 1
                .splineToConstantHeading(new Vector2d(26,  -45),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40,  -32),Math.toRadians(90))

                .lineToConstantHeading(new Vector2d(40,  -20))
                .splineToConstantHeading(new Vector2d(44,  -10),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(44,-55))

                //pickup 2
                .lineToConstantHeading(new Vector2d(44,-10))
                .splineToConstantHeading(new Vector2d(54,  -10),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(54,-55))

                //pickup 3
                .lineToConstantHeading(new Vector2d(47,  -10))
                .splineToConstantHeading(new Vector2d(62,  -10),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //set arm to HP
                })
                .lineToConstantHeading(new Vector2d(62,-50)) //also gets to HP
                .lineToConstantHeading(new Vector2d(hpX,hpY)) //also gets to HP
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //closeClaw
                })

                //score 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //set arm to score
                })
                .lineToConstantHeading(new Vector2d(6-scoreGap,  scoreY))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //set arm to score
                })
                .waitSeconds(0.1)
                //HP 2
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //closeClaw
                })
                //score 2
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //set arm to score
                })
                .lineToConstantHeading(new Vector2d(6-scoreGap*2,  scoreY))
                //HP 3
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //closeClaw
                })
                //score 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //set arm to score
                })
                .lineToConstantHeading(new Vector2d(6-scoreGap*3,  scoreY))
                //HP 4
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //closeClaw
                })
                //score 4
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //set arm to score
                })
                .lineToConstantHeading(new Vector2d(6-scoreGap*4,  scoreY))
                //park
                .lineToConstantHeading(new Vector2d(hpX,  hpY))


                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

    public void moveLift(int ticks){
        robot.outtake.outtakeSlideLeft.setTargetPosition(ticks);
        robot.outtake.outtakeSlideRight.setTargetPosition(ticks);
        robot.outtake.outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideLeft.setPower(1);
        robot.outtake.outtakeSlideRight.setPower(1);
    }
    public void moveExtendo(double pos){
        robot.intake.leftExtendo.setPosition(pos);
        robot.intake.rightExtendo.setPosition(pos);
    }

}