package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.teamcode.Robot;

import java.nio.ByteOrder;
import java.util.Arrays;

// High(ish) level drivers for APDS-9151

@Disabled
@I2cDeviceType
@DeviceProperties(name = "FASTD-REVColorSensorV3", xmlTag = "FAST_DRIVER_REVCOLORSENSORV3_IMPL")
/**
 * All functions here are <b>SYNCRONOUS</b> so watch out.
 */
public class REVColorSensorV3 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private enum Register {
        MAIN_CTRL(0x00),
        PS_LED(0x01),
        PS_PULSES(0x02),
        PS_MEAS_RATE(0x03),
        LS_MEAS_RATE(0x04),
        LS_Gain(0x05),
        PART_ID(0x06),
        MAIN_STATUS(0x07),
        PS_DATA_0(0x08),
        PS_DATA_1(0x09),
        LS_DATA_IR_0(0x0A),
        LS_DATA_IR_1(0x0B),
        LS_DATA_IR_2(0x0C),
        LS_DATA_GREEN_0(0x0D),
        LS_DATA_GREEN_1(0x0E),
        LS_DATA_GREEN_2(0x0F),
        LS_DATA_BLUE_0(0x10),
        LS_DATA_BLUE_1(0x11),
        LS_DATA_BLUE_2(0x12),
        LS_DATA_RED_0(0x13),
        LS_DATA_RED_1(0x14),
        LS_DATA_RED_2(0x15),
        INT_CFG(0x19),
        INT_PST(0x1A),
        PS_THRES_UP_0(0x1B),
        PS_THRES_UP_1(0x1C),
        PS_THRES_LOW_0(0x1D),
        PS_THRES_LOW_1(0x1E),
        PS_CAN_0(0x1F),
        PS_CAN_1(0x20), // MONKEY HAPPENED HERE
        LS_THRES_UP_0(0x21),
        LS_THRES_UP_1(0x22),
        LS_THRES_UP_2(0x23),
        LS_THRES_LOW_0(0x24),
        LS_THRES_LOW_1(0x25),
        LS_THRES_LOW_2(0x26),
        LS_THRES_VAR(0x27);

        public final int value;

        Register(int value) {
            this.value = value;
        }
    }

    public enum ControlFlag {
            INT_PS_SLEEP((byte)         0b01000000), // After interrupt from prox return to standby
            INT_LS_SLEEP((byte)         0b00100000), // After interrupt from light sensor return to standby
            SOFTWARE_RESET((byte)       0b00010000), // Restart
            RGB_ENABLED((byte)          0b00000100), // Enable RGB and IR channels
            LIGHT_SENSOR_ENABLED((byte) 0b00000010), // Enable light sensor
            PROX_SENSOR_ENABLED((byte)  0b00000001); // Enable prox sensor

            public final byte mask;

            ControlFlag(byte mask) {
                this.mask = mask;
            }
        }

    public static class ControlRequest {
        private byte value = 0b00000000;

        public byte value() {
            return value;
        }

        /**
         * All flags are set to 0 on construction
         */
        public ControlRequest() {}

        public void toggleFlag(ControlFlag flag) {
            value ^= flag.mask;
        }

        public ControlRequest enableFlag(ControlFlag flag) {
            value |= flag.mask;
            return this;
        }

        public boolean getFlag(ControlFlag flag) {
            return (value & flag.mask) != 0;
        }
    }

    public enum LEDPulseModFreq {
        k60Hz((byte)  0b00110000),
        k70Hz((byte)  0b01000000),
        k80Hz((byte)  0b01010000),
        k90Hz((byte)  0b01100000),
        k100Hz((byte) 0b01110000);

        public final byte mask;

        LEDPulseModFreq(byte mask) {
            this.mask = mask;
        }
    }

    public enum LEDCurrent {
        m2POINT5A((byte) 0b00000000),
        m5A((byte)       0b00000001),
        m10A((byte)      0b00000010),
        m25A((byte)      0b00000011),
        m50A((byte)      0b00000100),
        m75A((byte)      0b00000101),
        m100A((byte)     0b00000110),
        m125A((byte)     0b00000111);

        public final byte mask;

        LEDCurrent(byte mask) {
            this.mask = mask;
        }
    }

    public enum PSResolution {
        EIGHT((byte)  0b00000000, 8),
        NINE((byte)   0b00001000, 9),
        TEN((byte)    0b00010000, 10),
        ELEVEN((byte) 0b00011000, 11);

        public final byte mask;
        public final int depth;

        PSResolution(byte mask, int depth) {
            this.mask = mask;
            this.depth = depth;
        }
    }

    public enum PSMeasureRate {
        m6p25s((byte) 0b00000001), // 6.5ms
        m12p5s((byte) 0b00000010),
        m25s((byte)   0b00000011),
        m50s((byte)   0b00000100),
        m100s((byte)  0b00000101),
        m200s((byte)  0b00000110),
        m400s((byte)  0b00000111);

        public final byte mask;

        PSMeasureRate(byte mask) {
            this.mask = mask;
        }
    }

    public enum LSResolution {
        TWENTY((byte)    0b00000000, 20),
        NINETEEN((byte)  0b00010000, 19),
        EIGHTEEN((byte)  0b00100000, 18),
        SEVENTEEN((byte) 0b00110000, 17),
        SIXTEEN((byte)   0b01000000, 16),
        THIRTEEN((byte)  0b01010000, 13);

        public final byte mask;
        public final int depth;

        LSResolution(byte mask, int depth) {
            this.mask = mask;
            this.depth = depth;
        }
    }

    public enum LSMeasureRate {
        m25s((byte)   0b00000000),
        m50s((byte)   0b00000001),
        m100s((byte)  0b00000010),
        m200s((byte)  0b00000011),
        m500s((byte)  0b00000100),
        m1000s((byte) 0b00000101),
        m2000s((byte) 0b00000111);

        public final byte mask;

        LSMeasureRate(byte mask) {
            this.mask = mask;
        }
    }

    public enum LSGain {
        ONE((byte)      0b00000000),
        THREE((byte)    0b00000001),
        SIX((byte)      0b00000010),
        NINE((byte)     0b00000011),
        EIGHTEEN((byte) 0b00000100);

        public final byte value;

        LSGain(byte mask) {
            this.value = mask;
        }
    }

    private static final I2cAddr addr = I2cAddr.create7bit(0x52);
    private static final String tag = "FASTD-REVColorSensorV3";
    private ControlRequest opmode = new ControlRequest();
    private int psDepth = 8;
    private int lsDepth = 18;

    public REVColorSensorV3(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        registerArmingStateCallback(false);
        this.deviceClient.setI2cAddress(addr);
        this.deviceClient.engage();
        this.deviceClient.write8(Register.MAIN_CTRL.value, opmode.value());
    }

    public void sendControlRequest(ControlRequest request) {
        opmode = request;

        deviceClient.write8(Register.MAIN_CTRL.value, opmode.value());
    }

    public void configurePSLEDLighting(LEDPulseModFreq freq, LEDCurrent curret) { // FIXME better name is def needed
        deviceClient.write8(Register.PS_LED.value, freq.mask | curret.mask);
    }

    public void configurePSLEDPulses(byte pulses) {
        deviceClient.write8(Register.PS_PULSES.value, pulses);
    }

    public void configurePSLED(LEDPulseModFreq freq, LEDCurrent current, byte pulses) {
        deviceClient.write(Register.PS_LED.value, new byte[] {(byte) (freq.mask | current.mask), pulses});
    }

    /**
     * Check if the LS has new data to poll
     * <b>After a data read this will return false</b>
     * @return true for new data is ready
     */
    public boolean lsNewData() {
        return (0b00001000 & deviceClient.read8(Register.MAIN_STATUS.value)) > 0;
    }

    public void configurePS(PSResolution resolution, PSMeasureRate rate) {
        deviceClient.write8(Register.PS_MEAS_RATE.value, resolution.mask | rate.mask);
        psDepth = resolution.depth;
    }

    public void configureLSRecording(LSResolution resolution, LSMeasureRate rate) {
        deviceClient.write8(Register.LS_MEAS_RATE.value, resolution.mask | rate.mask);
        lsDepth = resolution.depth;
    }

    public void configureLSGain(LSGain gain) {
        deviceClient.write8(Register.LS_Gain.value, gain.value);
    }

    public void configureLS(LSResolution resolution, LSMeasureRate rate, LSGain gain) {
        deviceClient.write(Register.LS_MEAS_RATE.value, new byte[] {(byte) (resolution.mask | rate.mask), gain.value});
        lsDepth = resolution.depth;
    }

    /**
     * Read proximity sensor raw data
     * @return Distance RAW
     */
    public int readPS() {
        int bitsRead = (int) Math.ceil(psDepth / 8.0);
        RobotLog.ww(tag, "bitsRead: " + bitsRead);
        byte[] data = deviceClient.read(Register.PS_DATA_0.value, bitsRead);
        byte[] padded = new byte[2];
        Arrays.fill(padded, (byte) 1);
        System.arraycopy(data, 0, padded, 0, data.length);
        RobotLog.ww(tag, "data: " + Arrays.toString(padded));
        if (bitsRead > 1){ // We read the overflow bit oh no!
            padded[1] &= 0b11110111; // Chop if off
            RobotLog.ww(tag, "readPS recieved a bit overflow");
        }
        return TypeConversion.byteArrayToShort(padded, ByteOrder.LITTLE_ENDIAN);
    }

    private int readLSValueRaw(Register reg) {
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(deviceClient.read(reg.value, (int) Math.round(lsDepth / 8.0)), ByteOrder.LITTLE_ENDIAN));
    }

    private float readLSValue(Register reg) {
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
            deviceClient.read(reg.value, (int) Math.round(lsDepth / 8.0)),
            ByteOrder.LITTLE_ENDIAN)
        ) / (float) ((1 << lsDepth) - 1);
    }

    public float readLSIR() {
        return readLSValue(Register.LS_DATA_IR_0);
    }

    public float readLSGreen() {
        return readLSValue(Register.LS_DATA_GREEN_0);
    }

    public float readLSBlue() {
        return readLSValue(Register.LS_DATA_BLUE_0);
    }

    public float readLSRed() {
        return readLSValue(Register.LS_DATA_RED_0);
    }

    public int[] readLSRGBRAW() {
        // It's practically the same to read 16 bits, stop, and read another 16 bits than to just bulk read the same
        byte[] data = deviceClient.read(Register.LS_DATA_GREEN_0.value, 3 * 3);

        // Adjust for small readings
        int[] ret = new int[3];

        // Over here we are manually swapping endianness
        ret[0] = TypeConversion.byteArrayToShort(new byte[] {data[8], data[7], data[6]}); // Red
        ret[1] = TypeConversion.byteArrayToShort(new byte[] {data[2], data[1], data[0]}); // Green
        ret[2] = TypeConversion.byteArrayToShort(new byte[] {data[5], data[4], data[3]}); // Blue
        return ret;
    }

    /**
     * Synchronous read I2C device for 9 registers
     * @return int array of [red, green, blue]
     */
    public float[] readLSRGB() {
        int[] rgb = readLSRGBRAW();
        float[] ret = new float[] {rgb[0], rgb[1], rgb[2]};

        // Normalize to 0-1 (according to REV's official driver code)
        double mag = ret[0] + ret[1] + ret[2];
        if (mag != 0) {
            for (int i = 0; i < ret.length; i++) {
                ret[i] /= mag;
            }
        }

        return ret;
    }
    /**
     * Synchronous read I2C device for 9 registers
     * @return int array of [red, green, blue, alpha]
     */
    public float[] readLSRGBA() {
        float[] ret = new float[4];
        float[] rgb = readLSRGB();
        System.arraycopy(rgb, 0, ret, 0, rgb.length);

        // https://stackoverflow.com/a/56678483
        float[] rgbCopy = rgb.clone();
        for (int i = 0; i < rgbCopy.length; i++) {
            if (rgbCopy[i] > 0.04045) {
                rgbCopy[i] /= 12.92;
            } else {
                rgbCopy[i] = (float) Math.pow(((rgbCopy[i] + 0.055) / 1.055), 2.4);
            }
        }

        ret[3] = (float) (0.2126 * rgbCopy[0] + 0.7152 * rgbCopy[1] + 0.0722 * rgbCopy[2]);

        return ret;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Broadcom;
    }

    @Override
    public String getDeviceName() {
        return tag;
    }
}