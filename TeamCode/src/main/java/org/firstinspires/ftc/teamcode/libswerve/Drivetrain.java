package org.firstinspires.ftc.teamcode.libswerve;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Drivetrain {
    public final Localizer localizer;
    protected final SwerveModule[] modules;
    private final double maxTurnSpeed, maxVelocity;
    private final ElapsedTime timer = new ElapsedTime();
    public double speed = 0;

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
     * Used for internal drivetrain operations <b>Does Set angle calling for you as well as power</b>
     * @param strafeVector Vector for strafing direction
     * @param turnPower 0-1 scalar for vector powers (its weird)
     */
    public void setPower(Vector2 strafeVector, double turnPower) {
        Vector2[] finalVectors = new Vector2[modules.length];
        int highestMag = 0;

        // Use this to get the highest magnitude
        for (int i = 0; i < modules.length; i++) {
            Vector2 turnVector = new Vector2(modules[i].y, -modules[i].x);
            turnVector.norm();
            turnVector.mul(turnPower);
            Vector2 finalVector = Vector2.add(turnVector, strafeVector);
            finalVectors[i] = finalVector;

            if (finalVectors[highestMag].mag() < finalVector.mag()) {
                highestMag = i;
            }
        }

        for (int i = 0; i < modules.length; i++) { // Post normalization
            if (finalVectors[highestMag].mag() != 0) {
                finalVectors[i].mul(1 / finalVectors[highestMag].mag());
            }
            modules[i].fwdPower = Math.min(finalVectors[i].mag(), 1.0) * speed;
            // This preserves the angle. Makes it so when the stick isn't in a direction it will not update the module rotation.
            if (finalVectors[i].x != 0 || finalVectors[i].y != 0) {
                modules[i].setTargetAngle(Math.atan2(finalVectors[i].y, finalVectors[i].x));
            }
        }
    }

    /**
     * Pure pursuit implementation <code>c.check()</code> returns if the execution should continue or abort<br>
     * <b>WARNING: THIS FUNCTION DOES NOT UPDATE THE DRIVETRAIN. DO IT YOURSELF IN <code>c.update()</code></b><br>
     * @param p Path to follow
     * @param c <code>.check()</code> function checks if you should stop or not. <code>.update</code> should update all subsystems
     */
    public void executePurePursuit(Path p, OpmodeFunctions c, double speed) {
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
            /*for (SwerveModule module : modules) {
                // FIXME
                module.fwdPower = speed;
                module.setPowerDT(
                    strafeVector,
                    // Ill write some stuff on how I got this later
                    (maxVelocity * dist) / (maxTurnSpeed * target.h + maxVelocity * dist)
                );
            }*/

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
