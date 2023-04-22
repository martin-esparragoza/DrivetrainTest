package org.firstinspires.ftc.teamcode.libswerve;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Drivetrain {
    public final Localizer localizer;
    protected final SwerveModule[] modules;
    private final double maxTurnSpeed, maxVelocity;
    private final ElapsedTime timer = new ElapsedTime();
    protected final double speed;

    /**
     *
     * @param localizer
     * @param modules
     * @param maxVelocity The maximum velocity of the robot (in/ns)
     * @param maxTurnSpeed Max turn speed of robot (radians/ns)
     */
    public Drivetrain(double speed, Localizer localizer, SwerveModule[] modules, double maxVelocity, double maxTurnSpeed) {
        this.speed = speed;
        this.localizer = localizer;
        this.modules = modules;
        this.maxVelocity = maxVelocity;
        this.maxTurnSpeed = maxTurnSpeed;
    }

    public abstract void setModules(Gamepad gamepad);

    /**
     * <b>Updates all swerve modules</b>>
     */
    public void update() {
        for (SwerveModule module : modules) {
            module.update();
        }
    }

    /**
     * Pure pursuit implementation <code>c.check()</code> returns if the execution should continue or abort<br>
     * <b>WARNING: THIS FUNCTION DOES NOT UPDATE THE DRIVETRAIN. DO IT YOURSELF IN <code>c.update()</code></b><br>
     * @param p Path to follow
     * @param c <code>.check()</code> function checks if you should stop or not. <code>.update</code> should update all subsystems
     */
    public void executePurePursuit(Path p, OpmodeFunctions c) {
        timer.reset();

        while (!p.points.isEmpty() && c.check()) {
            Pose2d target = p.points.getFirst();
            Pose2d current = localizer.getPoseEstimate();

            // Remove points that we are close enough to
            double dist = target.distance(current);
            if (dist <= p.threshold) {
                p.points.removeFirst();
                target = p.points.getFirst();
                dist = target.distance(current);
            }

            Vector2 strafeVector = new Vector2(current.x - target.x, current.y - target.y);
            for (SwerveModule module : modules) {
                module.fwdPower = speed;
                Vector2 turnVector = new Vector2(module.y, -(module.x)); // Perp

                turnVector.mul(
                    // Ill write some stuff on how I got this later
                    (maxVelocity * dist) / (maxTurnSpeed * target.h + maxVelocity * dist)
                );

                Vector2 finalVector = Vector2.add(turnVector, strafeVector);

                module.setTargetAngle(Math.atan2(finalVector.y, finalVector.x));
            }

            c.update();
            timer.reset();
        }

        brake();
    }

    public void brake() {
        for (SwerveModule module : modules) {
            module.fwdPower = 0;
        }
    }
}
