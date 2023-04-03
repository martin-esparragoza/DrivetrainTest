package org.firstinspires.ftc.teamcode.libswerve;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class SwerveModule {
    private double targetAngle = 0;
    public static PID pid = new PID(0.5, 0, 0.1);
    public double fwdPower = 0;
    protected final double x, y;

    public SwerveModule(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Set powers of a swerve module
     * turning and forward
     * @param turnPower How fast to turn a swerve module [-1, 1]
     * @param forwardPower How fast to move the wheel on a swerve module [-1, 1]
     */
    public abstract void setPowers(double turnPower, double forwardPower);

    public abstract double getAngle();

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void update() {
        setPowers(-pid.getOut(Util.clampAngle(targetAngle - getAngle())), fwdPower);
    }
}