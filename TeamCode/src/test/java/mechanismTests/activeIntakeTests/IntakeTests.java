package mechanismTests.activeIntakeTests;

import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class IntakeTests {
    private Intake intake;
    private ActiveIntake activeIntake;

    // MOCKS
    @Mock
    Servo rightLinkage;
    @Mock
    Servo leftLinkage;
    @Mock
    Servo pivotServo;
    @Mock
    DcMotorEx rollerMotor;

    @BeforeEach
    public void setup() {
       activeIntake = new ActiveIntake(rollerMotor, pivotServo);
        intake = new Intake(rightLinkage, leftLinkage, activeIntake);
    }

    @Test
    public void testLinkagesDoExtendToFullPos() {
        intake.extendoFullExtend();
        // do it both ways to ensure same number being referenced
        verify(rightLinkage).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_EXTENDED.rLinkagePos());
        verify(leftLinkage).setPosition(.035);
    }

    @Test
    public void testLinkagesDoExtendToRetractedPos() {
        intake.extendoFullRetract();
        // do it both ways to ensure same number being referenced
        verify(rightLinkage).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.rLinkagePos());
        verify(leftLinkage).setPosition(.33);
    }

    @Test
    public void testLinkagesDoExtendForOuttaking() {
        intake.extendForOuttake();
        // do it both ways to ensure same number being referenced
        verify(rightLinkage).setPosition(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
        verify(leftLinkage).setPosition(.3);
    }

    @Test
    public void testLinkagesDoNotMoveToWrongPosWhenExtending() {
        intake.extendoFullExtend();
        verify(rightLinkage, never()).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.rLinkagePos());
        verify(leftLinkage, never()).setPosition(-0.8);
    }

    // BACK HERE ONCE DONE WITH ACTIVE
    @Test
    public void testIntakeResetHardware() {
        intake.resetHardware();
        verify(pivotServo).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.pivotPos());
        verify(rollerMotor).setPower(0);
        verify(rollerMotor).setDirection(DcMotorSimple.Direction.REVERSE);
        verify(rightLinkage).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.rLinkagePos());
        verify(leftLinkage).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.lLinkagePos());
    }

    @Test
    public void testPivotFlipsDown() {
        activeIntake.flipDownFull();
        verify(pivotServo).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_EXTENDED.pivotPos());
    }

    @Test
    public void testPivotFlipsDownToClear() {
        activeIntake.flipDownToClear();
        verify(pivotServo).setPosition(IntakeConstants.ActiveIntakeStates.CLEARING.pivotPos());
    }

    @Test
    public void testPivotFlipsUpToTransfer() {
        activeIntake.flipToTransfer();
        verify(pivotServo).setPosition(IntakeConstants.ActiveIntakeStates.TRANSFER.pivotPos());
    }

    @Test
    public void testPivotFlipsUp() {
        activeIntake.flipUp();
        verify(pivotServo).setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.pivotPos());
    }

    @Test
    public void motorTurnsOnToClear() {
        activeIntake.motorRollerOnToClear();
        verify(rollerMotor).setPower(0.7);
    }

    @Test
    public void motorTurnsOff() {
        activeIntake.motorRollerOff();
        verify(rollerMotor).setPower(0);
    }

    @Test
    public void motorTurnsOnToIntake() {
        activeIntake.motorRollerOnToIntake();
        verify(rollerMotor).setPower(-1);
    }

    @Test
    public void transferTurnsMotorOff() {
        activeIntake.transferOff();
        verify(rollerMotor).setPower(0);
    }

}
