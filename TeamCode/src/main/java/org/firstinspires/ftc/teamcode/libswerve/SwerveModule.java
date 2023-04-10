package org.firstinspires.ftc.teamcode.libswerve;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class SwerveModule {
    private double targetAngle = 0;
    public static PID pid = new PID(1, 0, 0.6);
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

    /**
     *
     * @return Hardware angle (Determined by you)
     */
    public abstract double getAngle();

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
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