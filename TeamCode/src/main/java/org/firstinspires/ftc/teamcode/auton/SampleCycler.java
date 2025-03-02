package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class SampleCycler extends LinearOpMode {
    ElapsedTime limeLightTimer = new ElapsedTime();
    Pose2d poseEstimate;
    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(-37.5, -63.5, Math.toRadians(0));
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //mechanisms
    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;

    //tunable pos
    public double scorePosX = -55.5; // TODO: tune
    public double scorePosY = -55.5;

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

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence initialSamples = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX-3, scorePosY, Math.toRadians(60)))
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
                .lineToLinearHeading(new Pose2d(-48,-54, Math.toRadians(90)))
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
                    intake.activeIntake.rollerMotor.setPower(0.45);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    moveLift(2000);
                    intake.activeIntake.transferOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1.2)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
//                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                    moveExtendo(.1);
                })

                //pickUp2
                .lineToLinearHeading(new Pose2d(-57, -54, Math.toRadians(90)))
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

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.45);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    intake.activeIntake.transferOff();
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .waitSeconds(1.2)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(0);
                    moveExtendo(IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos());
                })

                //pickUp3
                .lineToLinearHeading(new Pose2d(-30, -43, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    moveExtendo(0.1);
                })
                .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(170)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                })





                //score
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.45);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    arm.closeClaw();
                })
//                .turn(Math.toRadians(-125))
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intake.activeIntake.transferOff();
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(1.2)
                .build();


        currentState = State.initialSamplesState;
        drive.followTrajectorySequenceAsync(initialSamples);

        while (opModeIsActive() && !isStopRequested()) {


            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case initialSamplesState:
                case scoreState:
                    if (!drive.isBusy()) {
                        arm.openClaw();
                        arm.readyForTransfer();
                        moveLift(0);

                        currentState = State.pickupState;
                        pickUpPath(poseEstimate);
                    }
                    break;
                case pickupState:
                case spitState:
                    if (!drive.isBusy()) {
                        currentState = State.limelightState;
                        //TODO: start running limelight and start new elapsed timer for limelight
                        limeLightTimer.reset();
                    }
                    break;
                case limelightState:
                    //TODO: end limelight and get limelightOffsets save them to a variable
                    if (limeLightTimer.milliseconds() > 100) {
                        currentState = State.intakeState;
                        intakePath(poseEstimate, 0 ,0);
                    }
                    break;
                case intakeState:
                    if (!drive.isBusy()) {
                        if (true){ //TODO: replace with conditional for color sensor
                            currentState = State.scoreState;
                            scorePath(poseEstimate);
                        }else{
                            currentState = State.spitState;
                            spitPath(poseEstimate);
                        }
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();

            poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    enum State {
        initialSamplesState,
        pickupState,
        limelightState,
        intakeState,
        spitState,
        scoreState,
        IDLE            // Our bot will enter the IDLE state when done
    }

    public void pickUpPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .splineTo(new Vector2d(-35,-10), Math.toRadians(0))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void intakePath(Pose2d robotPose, int xOffset, int yOffset){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extendo correct amount, ask bee for how to get the perfect extension things for the axons.
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //flip down pivot, run intake
                })
                .lineToConstantHeading(new Vector2d(-30 + xOffset,-10 + yOffset)) // plus is for vision offsets
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void scorePath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //flip up pivot, stop intake, extendo in
                })
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //transfer jawn
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //slides up
                })
                .lineToLinearHeading(new Pose2d(-40, robotPose.getY(),Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(0.15)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void spitPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .lineToConstantHeading(new Vector2d(-35,-10))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
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