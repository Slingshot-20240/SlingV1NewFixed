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
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class SpecNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;

    public double hpX = 44;
    public double hpY = -68;
    public double scoreY = -32;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        arm = robot.arm;
        //specimenClaw = robot.specimenClaw;
        robot.intake.extendoFullRetract();
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        arm.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(13.25, -64.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preloaded spec
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(100);
                    arm.toScoreSpecimen();
                    //raise slides (small);
                    //flip arm to score
                })
                .lineToConstantHeading(new Vector2d(4,  scoreY-2))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                    moveLift(0);
                })

                //pickup 1
                .splineToConstantHeading(new Vector2d(26,  -45),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(39,  -32),Math.toRadians(90))


                .lineToConstantHeading(new Vector2d(39,  -13))
                .splineToConstantHeading(new Vector2d(43,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(44,  -13))
                .lineToConstantHeading(new Vector2d(44,-55))

                //pickup 2
                .lineToConstantHeading(new Vector2d(42,-13))
                .splineToConstantHeading(new Vector2d(53,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(53,-55))

                //pickup 3
                .lineToConstantHeading(new Vector2d(47,  -13))
                .splineToConstantHeading(new Vector2d(59.5,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(59.5,-50))

                //HP 1
                .lineToConstantHeading(new Vector2d(hpX-10,hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })

                //score 1
                .lineToConstantHeading(new Vector2d(-3,  scoreY-5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveLift(0);
                })
                //HP 2
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 2
                .lineToConstantHeading(new Vector2d(2,  scoreY-7.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveLift(0);
                })

                //HP 3
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 3
                .lineToConstantHeading(new Vector2d(-4,  scoreY-6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveLift(0);
                })
                //HP 4
                .lineToConstantHeading(new Vector2d(hpX,  hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 4
                .lineToConstantHeading(new Vector2d(-10,  scoreY-7.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveLift(0);
                })
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