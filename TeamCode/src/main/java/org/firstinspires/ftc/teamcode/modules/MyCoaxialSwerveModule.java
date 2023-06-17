package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.coaxial.CoaxialSwerveModule;

public class MyCoaxialSwerveModule extends CoaxialSwerveModule {
    /**
     * @param x Reversed
     * @param y Reversed
     * @param forward
     * @param turn
     * @param encoder
     */
    public MyCoaxialSwerveModule(double x, double y, DcMotor forward, CRServo turn, AnalogInput encoder) {
        super(x, y, forward, turn, encoder);
        forward.setDirection(DcMotor.Direction.FORWARD);
        forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public double getAngle() {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }
}
