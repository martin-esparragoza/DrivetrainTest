package org.firstinspires.ftc.teamcode.libswerve;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class Drivetrain {
    public final Localizer localizer;
    private final SwerveModule[] modules;
    private final double orgx, orgy;

    /**
     *
     * @param orgx X position for the robot centric origin (inches)
     * @param orgy Y position for the robot centric origin (inches)
     * @param localizer
     * @param modules
     */
    public Drivetrain(double orgx, double orgy, Localizer localizer, SwerveModule[] modules) {
        this.orgx = orgx;
        this.orgy = orgy;
        this.localizer = localizer;
        this.modules = modules;
    }

    /**
     * Gamepad left stick: strafing<br>
     * Gamepad right stick: turning<br>
     * <b>DOES NOT UPDATE THE SWERVE MODULES USE <code>Drivetrain.update()</code></b>
     * @param gamepad
     */
    public void setModules(Gamepad gamepad) {
        for (SwerveModule module : modules) {
            // Distace, use both gamepads to get the power
            module.fwdPower =
                Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) +
                Math.sqrt(Math.pow(gamepad.right_stick_y, 2) + Math.pow(gamepad.right_stick_x, 2));
            // Don't allow for it to go above 1
            if (Math.abs(module.fwdPower) / 2 > 1) {
                module.fwdPower /= 2;
            }

            double strafeAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
            // Perpendicular angle to robot origin
            double turnAngle = ((Math.PI / 2) + Math.atan2(module.y + orgy, module.x + orgx)) * Math.signum(gamepad.right_stick_x);

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
     * <b>Changes robot heading</b><br>
     * Pure pursuit implementation <code>c.call()</code> returns null<br>
     * <b>WARNING: THIS FUNCTION DOES NOT UPDATE THE DRIVETRAIN. DO IT YOURSELF IN <code>c.update()</code></b>
     * @param p Path to follow
     * @param c <code>.check()</code> function checks if you should stop or not. <code>.update</code> should update all subsystems
     */
    public void executePurePursuit(Path p, OpmodeFunctions c) {
    }

    public void brake() {
        for (SwerveModule module : modules) {
            module.fwdPower = 0;
        }
    }
}
