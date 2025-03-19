package fsmTests;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.mechanisms.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.misc.ReLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.misc.PIDFControllerEx;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class ActiveCycleTests {
    // intake hardware
    @Mock
    Servo rightExtendo;
    @Mock
    Servo leftExtendo;
    @Mock
    DcMotorEx rollerMotor;
    @Mock
    Servo pivotAxon;

    // outtake hardware
    @Mock
    DcMotorEx slideRight;
    @Mock
    DcMotorEx slideLeft;
    @Mock
    Servo bucketServo;
    PIDController controller = new PIDController(0, 0, 0);

    // arm
    @Mock
    Servo armPivot;
    @Mock
    Servo wrist;
    @Mock
    Servo claw;

    // drivetrain hardware
    @Mock
    DcMotorEx leftFront;
    @Mock
    DcMotorEx rightFront;
    @Mock
    DcMotorEx leftBack;
    @Mock
    DcMotorEx rightBack;
    @Mock
    IMU imu;
    @Mock
    ColorRangeSensor colorSensor;
    @Mock
    TouchSensor touchSensor;

    ColorSensorI2C colorSensorI2C;

    // other hardware
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    // -- actual objects --
    ActiveIntake activeIntake;
    Intake intake;
    Outtake outtake;
    DriveTrain drivetrain;
    Arm arm;

    // this may not work...
    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    ActiveCycle cycle;
    Robot robot;
    ElapsedTime loopTime;
    double startTime;

    @BeforeEach
    public void setUp() {
        activeIntake = new ActiveIntake(rollerMotor, pivotAxon);
        intake = new Intake(rightExtendo, leftExtendo, activeIntake);
        outtake = new Outtake(slideLeft, slideRight, controller, touchSensor);
        drivetrain = new DriveTrain(leftFront, rightFront, leftBack, rightBack, imu);
        colorSensorI2C = new ColorSensorI2C(colorSensor, true);
        arm = new Arm(armPivot, wrist, claw);

        robot = new Robot(controls, drivetrain, outtake, intake, colorSensorI2C, arm);
        cycle = new ActiveCycle(null, controls, robot);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }

    @Test
    public void testInRetractedToExtended() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);
        // push the extend button
        controls.extend.set(true);
        // loop!
        cycle.activeIntakeUpdate();
        // we should've moved to the fully extended state
        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED, cycle.getState());

        cycle.activeIntakeUpdate();

        verify(rightExtendo).setPosition(anyDouble());
        verify(leftExtendo).setPosition(anyDouble());

        verify(slideRight, times(2)).setPower(anyDouble());
        verify(slideLeft, times(2)).setPower(anyDouble());
    }

    @Test
    public void testInRetractedToTransfer() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);

        controls.extend.set(false);
        // hold transfer button
        controls.transfer.setLocked(true);

        cycle.activeIntakeUpdate();

        controls.transfer.setLocked(true);

        verify(rollerMotor).setPower(anyDouble());

        controls.transfer.setLocked(false);

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }


    @Test
    public void testInRetractedToHighBasket() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);

        controls.extend.set(false);
        // hold transfer button
        controls.transfer.setLocked(true);

        cycle.activeIntakeUpdate();

        controls.highBasket.set(true);

        verify(rollerMotor).setPower(anyDouble());

        controls.highBasket.set(false);

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }

    @Test
    public void testToSpecMode() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);

        controls.specMode.set(true);

        cycle.activeIntakeUpdate();

        cycle.startTime = -500;

        cycle.activeIntakeUpdate();

        verify(armPivot).setPosition(anyDouble());

        cycle.startTime = -1100;

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.SPEC_IDLE, cycle.getState());
    }

    @Test
    public void testToIntakeToSpec() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);

        controls.specMode.set(true);

        cycle.activeIntakeUpdate();

        cycle.startTime = -1100;

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.SPEC_IDLE, cycle.getState());

        controls.extend.set(true);

        cycle.activeIntakeUpdate();

        verify(rightExtendo, atLeastOnce()).setPosition(anyDouble());

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED, cycle.getState());

        controls.extend.set(false);

        cycle.activeIntakeUpdate();

        verify(rightExtendo, atLeastOnce()).setPosition(anyDouble());

        assertEquals(ActiveCycle.TransferState.SPEC_IDLE, cycle.getState());
    }

    @Test
    public void testToBaseStateFromTransfer() {
        cycle.setState(ActiveCycle.TransferState.TRANSFERING);
        controls.botToBaseState.set(true);
        cycle.activeIntakeUpdate();
        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }

    @Test
    public void testHang() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED);
        controls.hang.set(true);
        cycle.activeIntakeUpdate();
        assertEquals(ActiveCycle.TransferState.HANGING, cycle.getState());
    }

    @Test
    public void testRetractAndSampleMode() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED);
        controls.extend.set(false);
        cycle.activeIntakeUpdate();
        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }

    @Test
    public void testToIntaking() {
        cycle.setState(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED);
        controls.intakeOnToIntake.setLocked(true);
        cycle.activeIntakeUpdate();
        assertEquals(ActiveCycle.TransferState.INTAKING, cycle.getState());
    }

    @Test
    public void testFlipUpWhenNotLocked() {
        controls.extend.set(true);
        cycle.setState(ActiveCycle.TransferState.INTAKING);
        controls.intakeOnToIntake.setLocked(true);
        cycle.activeIntakeUpdate();
        controls.intakeOnToIntake.setLocked(false);
        cycle.activeIntakeUpdate();
        verify(pivotAxon, atLeastOnce()).setPosition(anyDouble());
        assertEquals(ActiveCycle.TransferState.INTAKING, cycle.getState());
    }

    @Test
    public void testClear() {
        cycle.setState(ActiveCycle.TransferState.INTAKING);
        controls.extend.set(true);

        controls.pivotToClear.setLocked(true);
        controls.clearIntake.set(true);
        cycle.activeIntakeUpdate();

        verify(rollerMotor).setPower(anyDouble());

        controls.clearIntake.set(false);
        cycle.activeIntakeUpdate();

        verify(rollerMotor).setPower(0);

        assertEquals(ActiveCycle.TransferState.INTAKING, cycle.getState());
    }

    @Test
    public void testTransferWorks() {
        cycle.setState(ActiveCycle.TransferState.TRANSFERING);
        controls.transfer.setLocked(true);
        cycle.activeIntakeUpdate();
        verify(rollerMotor).setPower(anyDouble());
        assertEquals(ActiveCycle.TransferState.TRANSFERING, cycle.getState());
    }

    @Test
    public void testHighBasketFromTransfer() {
        cycle.setState(ActiveCycle.TransferState.TRANSFERING);
        controls.highBasket.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.HIGH_BASKET, cycle.getState());
    }

    @Test
    public void testLowBasketFromTransfer() {
        cycle.setState(ActiveCycle.TransferState.TRANSFERING);
        controls.lowBasket.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.LOW_BASKET, cycle.getState());
    }
    @Test
    public void testTransferToExtend() {
        cycle.setState(ActiveCycle.TransferState.TRANSFERING);
        controls.extend.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED, cycle.getState());
    }
    @Test
    public void testHighbasketToRetracted() {
        cycle.setState(ActiveCycle.TransferState.HIGH_BASKET);
        controls.highBasket.set(false);
        cycle.activeIntakeUpdate();

        controls.highBasket.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }
    @Test
    public void testIdleToSpecScore() {
        controls.specMode.set(true);
        cycle.setState(ActiveCycle.TransferState.SPEC_IDLE);
        controls.scoreSpec.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.SPEC_SCORING, cycle.getState());
    }
    @Test
    public void testIdleToExtend() {
        controls.specMode.set(true);
        cycle.setState(ActiveCycle.TransferState.SPEC_IDLE);
        controls.extend.set(true);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED, cycle.getState());
    }
    @Test
    public void testIdleReturnToSampleMode() {
        cycle.setState(ActiveCycle.TransferState.SPEC_IDLE);
        controls.specMode.set(false);
        cycle.activeIntakeUpdate();

        cycle.startTime = -600;

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED, cycle.getState());
    }
    @Test
    public void testSpecScoringToSpecRetracting() {
        cycle.setState(ActiveCycle.TransferState.SPEC_SCORING);
        controls.scoreSpec.set(false);
        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.SPEC_RETRACTING, cycle.getState());
    }

    @Test
    public void testSpecRetractingToIdle() {
        cycle.setState(ActiveCycle.TransferState.SPEC_RETRACTING);

        cycle.startTime = -900;

        cycle.activeIntakeUpdate();

        assertEquals(ActiveCycle.TransferState.SPEC_IDLE, cycle.getState());
    }


}
