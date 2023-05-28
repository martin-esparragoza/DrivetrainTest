package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(xmlTag = "BNO055", name = "BNO055 (CUSTOM)", description = "Custom drivers for BNO055 IMU")
public class BNO055 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements IMU {
    public BNO055(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public BNO055(I2cDeviceSynch i2cDeviceSynch) {
        this(i2cDeviceSynch, false);
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }
}