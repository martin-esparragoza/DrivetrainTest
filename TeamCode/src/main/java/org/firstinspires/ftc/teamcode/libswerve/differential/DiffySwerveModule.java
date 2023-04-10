package org.firstinspires.ftc.teamcode.libswerve.differential;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;

public abstract class DiffySwerveModule extends SwerveModule {
    protected final DcMotor right;
    protected final DcMotor left;

    /**
     * Sets motors to RUN_WITHOUT_ENCODERS and zero power brake
     * @param x inches
     * @param y inches
     * @param right
     * @param left
     */
    public DiffySwerveModule(double x, double y, DcMotor right, DcMotor left) {
        super(x, y);
        this.left = left;
        this.right = right;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void setPowers(double turnPower, double forwardPower) {
        right.setPower(turnPower + forwardPower);
        left.setPower(turnPower - forwardPower);
    }
}
