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
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//TODO: merge and uncomment below
//import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
@Autonomous
public class SampleCyclerNewThingy extends LinearOpMode {
    ElapsedTime limeLightTimer = new ElapsedTime();
    Pose2d poseEstimate;
    State currentState = State.IDLE;
    SampleMecanumDrive drive;
    private int sample = 0;

    //mechanisms
    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;
    private ColorSensorI2C colorSensor;
    private Limelight limelight;


    //tunable pos
    public double scorePosX = -55;
    public double scorePosY = -55;

    //limelight
    boolean isBlue = true;

    // outtake
    public int slidePos = 275;

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
                    moveLift(1700);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX-1.5, scorePosY, Math.toRadians(74)))
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.motorRollerOnToIntake();
                    intake.activeIntake.flipDownFull();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
                    //intake.activeIntake.flipDownFull();
                })

                //pickUp1
                .lineToConstantHeading(new Vector2d(-50.5,-42))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
//                .waitSeconds(.1)
                .lineToLinearHeading(new Pose2d(scorePosX-1, scorePosY-1, Math.toRadians(78)))


                //score
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
//                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                })

                //pickUp2
                .lineToLinearHeading(new Pose2d(-63, -43, Math.toRadians(78)))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
                //.waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(scorePosX-1, scorePosY-1, Math.toRadians(75)))

                //score

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
                    //moveExtendo(IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos());
                })

                //pickUp3
//                .lineToLinearHeading(new Pose2d(-36, -44.75, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    moveExtendo(0.1);
                })
                .lineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })

                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                //.waitSeconds(0.6)
                //.lineToLinearHeading(new Pose2d(scorePosX+1, scorePosY+1, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(1)
                .build();

        while(opModeInInit() && !isStopRequested()){
            //TODO: add controller inputs here for alliance color
            //TODO: add stuff that changes what color we are  sensing for in our limelight
        }
        waitForStart();

        if (isStopRequested()) return;

        currentState = State.initialSamplesState;
        drive.followTrajectorySequenceAsync(trajSeq);
        sample = 4;

        while (opModeIsActive() && !isStopRequested()) {

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case initialSamplesState:
                case scoreState:
                    if(!drive.isBusy()) {
                        if (sample == 6) {
                            arm.openClaw();
                            currentState = State.parkState;
                            parkPath(poseEstimate);
                        } else {
                            arm.openClaw();
                            currentState = State.pickupState;
                            pickUpPath(poseEstimate);
                        }
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
                    intake.activeIntake.rollerMotor.setPower(0.4);
                    //TODO: end limelight and get limelightOffsets save them to a variable see below
                    if (limeLightTimer.milliseconds() > 200) {
                        currentState = State.intakeState;
                        intakePath(poseEstimate, 0 ,limelight.getVals()[1]);

                    }
                    break;
                case intakeState:
                    if (!drive.isBusy()) {
                        intake.activeIntake.flipToTransfer();
                        moveLift(slidePos);
                        intake.extendToTransfer();
                        if(!colorSensor.hasSample() || colorSensor.opposingColor()){
                            currentState = State.spitState;
                            intake.activeIntake.rollerMotor.setPower(0.65);
                            spitPath(poseEstimate);
                        }else{
                            currentState = State.scoreState;
                            scorePath(poseEstimate);
                        }
                    }
                    break;
                case parkState:
                    if (!drive.isBusy()) {
                        moveLift(200);
                        currentState = State.IDLE;
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
            telemetry.addData("LL offset", limelight.getVals()[1]);
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
        parkState,
        IDLE            // Our bot will enter the IDLE state when done
    }

    public void pickUpPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(0);
                    arm.readyForTransfer();
                })
                .splineTo(new Vector2d(-20,-8), Math.toRadians(0))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void intakePath(Pose2d robotPose, double xOffset, double yOffset){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .lineToLinearHeading(new Pose2d(-20, robotPose.getY() + yOffset, Math.toRadians(0))) // plus is for vision offsets
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.flipDownFull();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.extendoFullExtend();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(.4)
                .turn(Math.toRadians(3))
                .turn(Math.toRadians(-3))
                .turn(Math.toRadians(3))
                .turn(Math.toRadians(-3))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void scorePath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                //transfer
                .addTemporalMarker(0.9, () -> {
                    moveLift(slidePos);
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })

                .addTemporalMarker(1.3, () -> {
                    arm.closeClaw();

                })
                .addTemporalMarker(1.5, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1900);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(-40, robotPose.getY(),Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(scorePosX-0.7, scorePosY-0.7, Math.toRadians(45)))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void spitPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .lineToConstantHeading(new Vector2d(-20,-8))
                .waitSeconds(0.4)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void parkPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(500);
                    arm.toScoreSpecimen();
                })
                .splineToLinearHeading(new Pose2d(-18,-8, Math.toRadians(180)), Math.toRadians(0))
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