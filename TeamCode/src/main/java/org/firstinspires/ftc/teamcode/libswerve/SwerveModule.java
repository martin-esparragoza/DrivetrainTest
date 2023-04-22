package org.firstinspires.ftc.teamcode.libswerve;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class SwerveModule {
    private double targetAngle = 0;
    public static PID pid = new PID(1, 0, 0.6);
    public double fwdPower = 0;
    public final double x;
    public final double y;

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

    /**
     *
     * @return Hardware angle (Determined by you)
     */
    public abstract double getAngle();

    /**
     * <b>CLIPS FOR YOU. SET THE POWER FIRST.</b>
     * @param targetAngle radians
     */
    public void setTargetAngle(double targetAngle) {
        // Instead of clamping to -180, 180 clamp to -90, 90 and just switch the signs on the fwdPower when necessary
        while (targetAngle > Math.PI / 2) {
            targetAngle -= Math.PI;
        }
        while (targetAngle < -(Math.PI / 2)) {
            targetAngle += Math.PI;
        }
        this.targetAngle = targetAngle;
        fwdPower *= Math.signum(targetAngle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Move to target angle
     */
    public void update() {
        // Set this to negative if PID absolutely not working
        setPowers(pid.getOut(Util.clampAngle(targetAngle - getAngle())), fwdPower);
    }
}