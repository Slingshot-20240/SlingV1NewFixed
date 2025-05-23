package org.firstinspires.ftc.teamcode.auton.archive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
public class SampNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;

    public double scorePosX = -55; // TODO: tune
    public double scorePosY = -55;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        arm = robot.arm;
        //specimenClaw = robot.specimenClaw;
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();
        arm.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-38.5, -63.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(60)))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
                    moveExtendo(.1);
                    //intake.activeIntake.flipDownFull();
                })

                //pickUp1
//                .turn(Math.toRadians(pick1Angle))
                .lineToLinearHeading(new Pose2d(-46.5,-57, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.motorRollerOnToIntake();
                    intake.activeIntake.flipDownFull();
                })
                .forward(15)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                })


                //score
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    arm.pullBackToGoUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1.7)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
//                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                    moveExtendo(.1);
                })

                //pickUp2
                .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .forward(15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                })
                .waitSeconds(.4)

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    arm.pullBackToGoUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1.6)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
                    moveExtendo(IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos());
                })

                //pickUp3
                .lineToLinearHeading(new Pose2d(-29.5, -43.25, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    moveExtendo(0.1);
                })
                .lineToLinearHeading(new Pose2d(-42.5, -31, Math.toRadians(170)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                })
                .waitSeconds(0.3)





                //score
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    arm.pullBackToGoUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(0.8)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    arm.openClaw();
//                })
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    arm.openClaw();
                })
                .waitSeconds(.1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
                })
                .forward(10)
                .build();

        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        moveLift(0);

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