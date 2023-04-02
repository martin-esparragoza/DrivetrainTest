package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;

// TODO
public class MySwerveModule extends DiffySwerveModule {
    private final AnalogInput encoder;

    public MySwerveModule(DcMotor right, DcMotor left, AnalogInput encoder) {
        super(right, left);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.encoder = encoder;
    }

    @Override
    public double getAngle() {
        return 0;
    }
}