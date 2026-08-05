package org.firstinspires.ftc.teamcode.kronbot.utils.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.kronbot.utils.devices.PCF8574;


/**
 * Driver class that uses a PCF8574 device to control a 2004 LCD module.
 */
public class I2cLcd {

    private static final byte RS_BIT = 1;
    private static final byte RW_BIT = 2;
    private static final byte EN_BIT = 4;
    private static final byte BL_BIT = 8;

    private final PCF8574 pcf;
    private byte backlight = 0;

    public I2cLcd(HardwareMap hardwareMap, String pcfName) {
        pcf = hardwareMap.get(PCF8574.class, pcfName);
    }

    /**
     * Print a string to the display. Only call this after calling setCursor().
     * @param text The string. Make sure it doesn't overflow the row.
     */
    public void print(String text) {
        for(int i = 0; i < text.length(); i++) {
            writeData((byte)text.charAt(i));
        }
    }

    /**
     * Set the cursor on the screen. Call this when writing on a new row, unless you want more advanced behavior
     * @param row The row [0,3]
     * @param col The column [0, 19]
     */
    public void setCursor(int row, int col) {
        int address = col;
        switch (row) {
            case 1:
                address += 40;
                break;
            case 2:
                address += 20;
                break;
            case 3:
                address += 60;
                break;
            default:
                break;
        }
        writeCommand((byte)(0b10000000 | (address & 0x7F)));
    }

    /**
     * Initialize the LCD. Can take up to 15ms.<br>
     * It is preferred to call this only once in init,
     * if you want to clear the screen, use clear().
     * @param cursor If true, will show a cursor on the LCD where the next character written will appear
     * @param cursorBlink If true, and if cursor is true, the cursor will blink
     * @param backlight If true, the backlight will be turned on from init
     */
    public void initLcd(boolean cursor, boolean cursorBlink, boolean backlight) {
        sendInitCmd();
        try {
            Thread.sleep(4);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
        sendInitCmd();
        sendInitCmd();
        send4bitInit();

        setBacklight(backlight);

        writeCommand((byte)0b00101000);     // Set 4 bit mode, 2 lines and 5x8 font
        if(cursor && cursorBlink)           // Set display on, with optional cursor and blinking
            writeCommand((byte)0b00001111);
        else if(cursor)
            writeCommand((byte)0b00001110);
        else
            writeCommand((byte)0b00001100);
        writeCommand((byte)0b00000001);     // Clear the display
        writeCommand((byte)0b00000010);     // Return the cursor home (top left)
        writeCommand((byte)0b00000110);     // Set the cursor to increment with each character
    }

    /**
     * Initialize the LCD. Can take up to 15ms.<br>
     * It is preferred to call this only once in init,
     * if you want to clear the screen, use clear().
     */
    public void initLcd() {
        initLcd(false, false, true);
    }

    /**
     * Write a byte of data to the LCD
     * @param data Refer to the HD44780 datasheet for more details
     */
    protected void writeData(byte data) {
        /* Wait for the lcd to not be busy */
        while(busy());

        byte snd = (byte)((data & 0xF0 | RS_BIT | backlight));
        pcf.writeByte(snd | EN_BIT);

        snd = (byte)((data << 4) | RS_BIT | backlight);
        pcf.writeByte(snd);
        pcf.writeByte(snd | EN_BIT);
        pcf.writeByte(snd);
    }

    /**
     * Write a command to the LCD
     * @param cmd Refer to the HD44780 datasheet for more details
     */
    protected void writeCommand(byte cmd) {
        /* Wait for the lcd to not be busy */
        while(busy());

        byte snd = (byte)((cmd & 0xF0) | backlight);
        pcf.writeByte(snd | EN_BIT);

        snd = (byte)((cmd << 4) | backlight);
        pcf.writeByte(snd);
        pcf.writeByte(snd | EN_BIT);
        pcf.writeByte(snd);
    }

    protected boolean busy() {
        /* Extract the busy flag from the read data */
        return (read() & 0x80) != 0;
    }

    /**
     * Read the busy flag and the address counter from the LCD
     * @return The read data (BF[7] and AC[6:0])
     */
    protected byte read() {
        byte snd = (byte)(0b11110000 | RW_BIT | backlight);
        byte readData = 0;

        pcf.writeByte(snd);
        pcf.writeByte(snd | EN_BIT);
        readData = (byte)(pcf.readByte() & 0xF0);
        pcf.writeByte(snd);
        pcf.writeByte(snd | EN_BIT);
        readData = (byte)(pcf.readByte() >> 4);
        pcf.writeByte(snd);

        return readData;
    }

    /**
     * Sets the backlight value. Will update on the next call that writes to the screen, unless immediate is true.
     * @param backlightOn If true, the backlight will turn on, and vice versa.
     * @param immediate If true, will send a command that turns on the backlight.
     *                  Do not use immediate while communication is undergoing.
     */
    public void setBacklight(boolean backlightOn, boolean immediate) {
        if(backlightOn)
            backlight = BL_BIT;
        if(immediate)
            pcf.writeByte(BL_BIT);
    }

    /**
     * Sets the backlight value. Will update on the next call that writes to the screen.
     * @param backlightOn If true, the backlight will turn on, and vice versa.
     */
    public void setBacklight(boolean backlightOn) {
        setBacklight(backlightOn, false);
    }

    private void sendInitCmd() {
        pcf.writeByte(0b00110000);
        pcf.writeByte(0b00110000 | EN_BIT);
        pcf.writeByte(0b00110000);
    }

    private void send4bitInit() {
        pcf.writeByte(0b00100000);
        pcf.writeByte(0b00100000 | EN_BIT);
        pcf.writeByte(0b00100000);
    }

    /** Changes the address of the device. Call if the address is not the default (0x27) */
    public void changeI2cAddress(byte new7bitAddress) {
        pcf.getDeviceClient().setI2cAddress(I2cAddr.create7bit(new7bitAddress));
    }
}
