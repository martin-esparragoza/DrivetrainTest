package org.firstinspires.ftc.teamcode.libswerve.differential;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;

public abstract class DiffySwerveModule extends SwerveModule {
    protected final DcMotor right;
    protected final DcMotor left;

    public DiffySwerveModule(double x, double y, DcMotor right, DcMotor left) {
        super(x, y);
        this.left = left;
        this.right = right;
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void setPowers(double turnPower, double forwardPower) {
        double rightPower = turnPower + forwardPower;
        double leftPower = turnPower - forwardPower;
        // Minimize to a power that won't go above 2
        if (Math.abs(rightPower) / 2 > 1.0 && Math.abs(leftPower) / 2 > 1.0) {
            rightPower /= 2;
            leftPower /= 2;
        }

        right.setPower(rightPower);
        left.setPower(leftPower);
    }
}
