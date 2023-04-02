package org.firstinspires.ftc.teamcode.libswerve.differential;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.PID;
import org.firstinspires.ftc.teamcode.libswerve.Util;

@Config
public abstract class SwerveModule {
    protected final DcMotor right;
    protected final DcMotor left;
    private double targetAngle = 0;
    public static PID pid = new PID(0.004, 0, 0.1);
    public double fwdPower = 0;

    public SwerveModule(DcMotor right, DcMotor left) {
        this.left = left;
        this.right = right;
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Set powers of a swerve module
     * turning and forward
     * @param turnPower How fast to turn a swerve module [-1, 1]
     * @param forwardPower How fast to move the wheel on a swerve module [-1, 1]
     */
    public final void setPowers(double turnPower, double forwardPower) {
        double rightPower = turnPower + forwardPower;
        double leftPower = turnPower - forwardPower;
        // Maximize the possible power that can be given to a motor
        if (Math.abs(rightPower) / 2 > 1.0 && Math.abs(leftPower) / 2 > 1.0) {
            rightPower /= 2;
            leftPower /= 2;
        }

        right.setPower(rightPower);
        left.setPower(leftPower);
    }

    public abstract double getAngle();

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        pid.reset();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void update() {
        setPowers(/*-pid.getOut(Util.clampAngle(targetAngle - getAngle()))*/0, fwdPower);
    }
}