package org.firstinspires.ftc.teamcode.libswerve;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class Drivetrain {
    public final Localizer localizer;
    private final SwerveModule[] modules;
    private final double orgx, orgy;
    private final double maxTurnSpeed, maxVelocity;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     *
     * @param orgx X position for the robot centric origin (inches)
     * @param orgy Y position for the robot centric origin (inches)
     * @param localizer
     * @param modules
     * @param maxVelocity The maximum velocity of the robot (in/ns)
     * @param maxTurnSpeed Max turn speed of robot (radians/ns)
     */
    public Drivetrain(double orgx, double orgy, Localizer localizer, SwerveModule[] modules, double maxVelocity, double maxTurnSpeed) {
        this.orgx = orgx;
        this.orgy = orgy;
        this.localizer = localizer;
        this.modules = modules;
        this.maxVelocity = maxVelocity;
        this.maxTurnSpeed = maxTurnSpeed;
    }

    /**
     * Gamepad left stick: strafing<br>
     * Gamepad right stick: turning<br>
     * <b>DOES NOT UPDATE THE SWERVE MODULES USE <code>Drivetrain.update()</code></b>
     * @param gamepad
     */
    public void setModules(Gamepad gamepad) {
        for (SwerveModule module : modules) {
            // Distance, use both gamepads to get the power
            double leftmag = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2));
            double rightmag = Math.sqrt(Math.pow(gamepad.right_stick_y, 2) + Math.pow(gamepad.right_stick_x, 2));
            module.fwdPower = Math.min(leftmag + rightmag, 1);

            double strafeAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
            // Perpendicular angle to robot origin
            double turnAngle =
                ((Math.PI / 2) + Math.atan2(module.y + orgy, module.x + orgx)) *
                Math.signum(gamepad.right_stick_x) *
                Math.max(1.0 - leftmag, 0.5 * rightmag); // If both left and right are used turn at half speed in order to also move

            module.setTargetAngle(Util.clampAngle(strafeAngle + turnAngle));
        }
    }

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

            for (SwerveModule module : modules) {
                module.fwdPower = 1.0;

                // These are some pretty sketchy things FIXME later
                double strafeAngle = Math.atan2(orgx + current.y - target.y, orgy + current.x - target.x);

                double turnProportion = (maxVelocity * dist) / (maxTurnSpeed * target.h + maxVelocity * dist);

                // Convert that to an angle!
                double turnAngle = ((Math.PI / 2) + Math.atan2(module.y + orgy, module.x + orgx)) * turnProportion;

                module.setTargetAngle(Util.clampAngle(strafeAngle + turnAngle));
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
