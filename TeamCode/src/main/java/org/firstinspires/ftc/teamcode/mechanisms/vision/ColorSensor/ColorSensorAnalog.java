package org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
//TODO: NOT USED
public class ColorSensorAnalog {
    public final RevColorSensorV3 emulator;
    private final I2cDeviceSynchSimple i2c;
    private final AnalogInput pin0;
    public ColorSensorAnalog(HardwareMap hardwareMap){
        //set up the emulator and the transmitter (i2c is only used for writing)
        this.emulator = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
        //analog mode, hsv - test lol
        this.setPin0Analog(AnalogMode.HSV);
        pin0 = hardwareMap.analogInput.get("analog0");
    }

    //i think this will work. i'm not sure

    public double getHue(){
        return pin0.getVoltage() / 3.3 * 360;
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
        return ((hue360 - 180) % 360);
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}