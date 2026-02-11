package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.kronbot.utils.devices.PCF8574.PCF8574;

/**
 * Uses two PCF8574 I/O expanders to control an LED bar
 */
public class I2cLedBar {
    PCF8574 pcf0;
    PCF8574 pcf1;

    /** Set what bits control consecutive LEDs from the first PCF8574 */
    static int[] ledBits0 = {
            0b00010000, // LED 1
            0b00000010, // LED 2
            0b00100000, // LED 3
            0b00000001, // LED 4
            0b01000000, // LED 5
            0b00000100, // LED 6
            0b10000000, // LED 7
    };
    /** Set what bits control consecutive LEDs from the second PCF8574 */
    static int[] ledBits1 = {
            0b00010000, // LED 8
            0b00000001, // LED 9
            0b01000000, // LED 10
            0b00000010, // LED 11
            0b00100000, // LED 12
    };

    public I2cLedBar(HardwareMap hardwareMap, String name0, int address7Bit0, String name1, int address7Bit1) {
        pcf0 = hardwareMap.get(PCF8574.class, name0);
        pcf0.getDeviceClient().setI2cAddress(I2cAddr.create7bit(address7Bit0));

        pcf1 = hardwareMap.get(PCF8574.class, name1);
        pcf1.getDeviceClient().setI2cAddress(I2cAddr.create7bit(address7Bit1));

        // Turn all LEDs on
        pcf0.writeByte(0x00);
        pcf1.writeByte(0x00);
    }

    /**
     * Set a number of consecutive LEDs on
     * @param val the number of LEDs to turn on, stating from the green LEDs. (from 0 to 12)
     */
    public void setLeds(int val) {
        int write0 = 0;
        int write1 = 0;

        if(val >= 0) {
            int i = 0;
            while(i < ledBits0.length && i < val) {
                write0 |= ledBits0[i];
                i++;
            }
            while(i < 12 && i < val) {
                write1 |= ledBits1[i - ledBits0.length];
                i++;
            }
        }

        // Reverse the bits since bits set to 0 turn LEDs on
        pcf0.writeByte((byte)(~write0));
        pcf1.writeByte((byte)(~write1));
    }

    /**
     * Show a value on the LED bar. When the actual value reaches the target value, one red LED will be on.
     * Otherwise, the bar will show the actual value down to 70% and up to 115% of the target value.
     * @param targetVal The value that will be centered on the first red LED
     * @param actualVal The value that will be shown on the bar
     */
    public void showValue(double targetVal, double actualVal) {
        showValue(targetVal, actualVal, 0.7);
    }
    /**
     * Show a value on the LED bar. When the actual value reaches the target value, one red LED will be on.
     * Otherwise, the bar will show the actual value down to relMult * targetVal and up to (1 + relMult / 2) * targetval
     * @param targetVal The value that will be centered on the first red LED
     * @param actualVal The value that will be shown on the bar
     * @param relMult The multiplier that will set the scale for the bar.
     *                The overload of this function withou this parameter uses a default value of 0.7
     */
    public void showValue(double targetVal, double actualVal, double relMult) {
        targetVal = Math.abs(targetVal);
        actualVal = Math.abs(actualVal);
        double minVal = targetVal * relMult;
        double deltaT = targetVal - minVal;
        double deltaA = actualVal - minVal;
        double relative = deltaA / deltaT;
        int LEDs = (int)Math.round(relative * 9);
        LEDs = Math.min(Math.max(LEDs, 0), 12);
        setLeds(LEDs);
    }
}
