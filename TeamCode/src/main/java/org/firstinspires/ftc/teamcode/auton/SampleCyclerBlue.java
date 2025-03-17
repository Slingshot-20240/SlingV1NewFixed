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
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//TODO: merge and uncomment below
//import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
@Autonomous
public class SampleCyclerBlue extends LinearOpMode {
    ElapsedTime limeLightTimer = new ElapsedTime();
    Pose2d poseEstimate;
    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    //mechanisms
    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;
    private ColorSensorI2C colorSensor;
    private Limelight limelight;


    //tunable pos
    public double scorePosX = -56;
    public double scorePosY = -56;

    //limelight
    boolean isBlue = true;

    // outtake
    public int slidePos = 300;

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

        drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-38.5, -63.5, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        colorSensor = robot.colorSensorI2C;
        colorSensor.setIsBlue(isBlue);
        limelight = new Limelight(hardwareMap, !isBlue, isBlue, true);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(60)))
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.2)
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
                    moveLift(slidePos);
                })
                .waitSeconds(.1)


                //score
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1.1)
                .lineToLinearHeading(new Pose2d(scorePosX - .5, scorePosY - .5, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.2)
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
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
                .waitSeconds(.2)

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1.2)
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
                .lineToLinearHeading(new Pose2d(-29.5, -42.75, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    moveExtendo(0.1);
                })
                .lineToLinearHeading(new Pose2d(-42.5, -31, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
                .waitSeconds(0.6)

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.pullBackToGoUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(scorePosX+.5, scorePosY+.5, Math.toRadians(45)))
                .waitSeconds(0.1)
                .build();

        while(opModeInInit() && !isStopRequested()){
            //TODO: add controller inputs here for alliance color
            //TODO: add stuff that changes what color we are  sensing for in our limelight
        }
        waitForStart();

        if (isStopRequested()) return;

        currentState = State.initialSamplesState;
        drive.followTrajectorySequenceAsync(trajSeq);

        while (opModeIsActive() && !isStopRequested()) {

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case initialSamplesState:
                case scoreState:
                    if (!drive.isBusy()) {
                        arm.openClaw();

                        currentState = State.pickupState;
                        pickUpPath(poseEstimate);
                    }
                    break;
                case pickupState:
                case spitState:
                    if (!drive.isBusy()) {
                        currentState = State.limelightState;
                        limeLightTimer.reset();
                    }
                    break;
                case limelightState:
                    moveExtendo(0.13);
                    intake.activeIntake.flipDownToClear();
                    intake.activeIntake.rollerMotor.setPower(0.75);
                    //TODO: end limelight and get limelightOffsets save them to a variable see below
                    if (limeLightTimer.milliseconds() > 100) {
                        currentState = State.intakeState;
                        intakePath(poseEstimate, 0 ,10);

//                        intakePath(poseEstimate, limelight.location()[0], limelight.location()[1]);
                    }
                    break;
                case intakeState:
                    if (!drive.isBusy()) {
                        intake.activeIntake.flipToTransfer();
                        moveLift(slidePos);
                        intake.extendToTransfer();
                        if(!colorSensor.hasSample() || colorSensor.opposingColor()){
                            currentState = State.spitState;
                            intake.activeIntake.rollerMotor.setPower(1);
                            spitPath(poseEstimate);
                        }else{
                            currentState = State.scoreState;
                            scorePath(poseEstimate);
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
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(0);
                    arm.readyForTransfer();
                })
                .splineTo(new Vector2d(-22,-8), Math.toRadians(0))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void intakePath(Pose2d robotPose, double xOffset, double yOffset){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .lineToConstantHeading(new Vector2d(-22, robotPose.getY() + yOffset)) // plus is for vision offsets
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.flipDownFull();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.extendoFullExtend();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(0.7)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void scorePath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                //transfer
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    arm.closeClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(2000);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-40, robotPose.getY(),Math.toRadians(0)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(0.1)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void spitPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .lineToConstantHeading(new Vector2d(-22,-8))
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