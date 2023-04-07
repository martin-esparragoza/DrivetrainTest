package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;

// TODO
public class MySwerveModule extends DiffySwerveModule {
    public MySwerveModule(double x, double y, DcMotor right, DcMotor left) {
        super(x, y, right, left);
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public double getAngle() {
        return (((right.getCurrentPosition() / 537.6) * (2 * Math.PI)) + ((left.getCurrentPosition() / 537.6) * (2 * Math.PI))) / (66.0 / 30.0) / 2;
    }
}