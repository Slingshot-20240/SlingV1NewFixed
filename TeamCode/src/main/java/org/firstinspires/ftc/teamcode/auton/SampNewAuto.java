package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class SampNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private static IntakeConstants.ActiveIntakeStates activeIntakeStates;
    private Intake intake;
    private Outtake outtake;

    public static double scorePosX = -53;
    public static double scorePosY = -53;

    public static double pick1Angle = 87;
    public static double pick2Angle = 110;
    public static double pick3Angle = 143;

    //TODO: when implementing extendo positions, use these to make it configurable
    public static double pick1Ext = 0;
    public static double pick2Ext = 0;
    public static double pick3Ext = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        outtake = robot.outtake;
        //specimenClaw = robot.specimenClaw;
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-37.5, -63.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slides
                    //flipArm
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)

                //pickUp1
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(pick1Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                    //intake Run
                })
                .waitSeconds(0.15)

                //pickUp2
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(pick2Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)

                //pickUp3
                .lineToLinearHeading(new Pose2d(-47, -43, Math.toRadians(pick3Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })

                //score
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)
                .forward(4)
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