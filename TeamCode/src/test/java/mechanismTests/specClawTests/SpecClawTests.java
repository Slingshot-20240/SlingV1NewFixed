package mechanismTests.specClawTests;
import static org.mockito.Mockito.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class SpecClawTests {
    private SpecimenClaw specClaw;

    @Mock
    Servo clawServo;
    @BeforeEach
    public void setUp() {
        specClaw = new SpecimenClaw(clawServo);
    }

    @Test
    public void testOpenClaw() {
        specClaw.openClaw();
        verify(clawServo).setPosition(SpecimenClaw.SpecConstants.OPEN.getClawPos());
    }

    @Test
    public void testCloseClaw() {
        specClaw.closeClaw();
        verify(clawServo).setPosition(SpecimenClaw.SpecConstants.CLOSED.getClawPos());
    }
}
