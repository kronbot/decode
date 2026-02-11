package org.firstinspires.ftc.teamcode.kronbot.utils.devices.PCF8574;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * A PCF8574 driver.
 * This device doesn't use any registers, you only ever read or write one byte.
 * (or more for consecutive writes/reads, but I don't use that here)
 */
@I2cDeviceType
@DeviceProperties(
        name = "PCF8574 Device",
        xmlTag = "PCF8574DEV",
        description ="PCF8574 Device"
)
public class PCF8574 extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    private static final byte DEFAULT_ADDRESS = 0x27;


    /**
     * Write a byte to the PCF8574.<br>
     * For more information, refer to the PCF8574 datasheet.
     */
    public void writeByte(int value) {
        deviceClient.write8(value & 0xFF);
    }


    /**
     * Write a byte to the PCF8574.<br>
     * For more information, refer to the PCF8574 datasheet.
     */
    public int readByte() {
        return deviceClient.read8();
    }


    /**
     * Used by the hardware map.
     * Example declaration:<br>
     * PCF8574 pcf = hardwareMap.get(PCF8574.class, "pcf");
     */
    public PCF8574(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        // The max frequency is 100kHz;
        // Refer to the datasheet for more information
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "PCF8574 Device";
    }
}