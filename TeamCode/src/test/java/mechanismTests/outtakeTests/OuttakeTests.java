package mechanismTests.outtakeTests;

import static org.mockito.Mockito.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class OuttakeTests {
    private Outtake outtake;
    private PIDController controller = new PIDController(0.1, 0, 0.0001);

    // MOCKS
    @Mock
    DcMotorEx outtakeSlideLeft;
    @Mock
    DcMotorEx outtakeSlideRight;
    @Mock
    Servo bucketServo;
    @Mock
    TouchSensor touchSensor;

    @BeforeEach
    public void setUp() {
        outtake = new Outtake(outtakeSlideLeft, outtakeSlideRight, controller, touchSensor);
    }

    @Test
    public void testMoveTicks() {
        outtake.moveTicks(1000);
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }

    @Test
    public void canExtendToHighBasket() {
        outtake.extendToHighBasket();
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }

    @Test
    public void canExtendToLowBasket() {
        outtake.extendToLowBasket();
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }

    @Test
    public void canExtendSpecHighRack() {
        outtake.extendToSpecimenHighRackLow();
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }

    @Test
    public void canReturnToRetracted() {
        outtake.returnToRetracted();
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }

    @Test
    public void testResetEncoders() {
        outtake.resetEncoders();
        verify(outtakeSlideLeft).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verify(outtakeSlideRight).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verify(outtakeSlideLeft).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        verify(outtakeSlideRight).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Test
    public void canGoToTransfer() {
        outtake.upToTransfer();
        verify(outtakeSlideLeft).setPower(anyDouble());
        verify(outtakeSlideRight).setPower(anyDouble());
    }
}
