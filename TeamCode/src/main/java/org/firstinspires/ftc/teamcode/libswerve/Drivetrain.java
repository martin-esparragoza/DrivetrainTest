package org.firstinspires.ftc.teamcode.libswerve;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {
    private final Localizer localizer;
    private final SwerveModule[] modules;
    private final double orgx, orgy;

    /**
     *
     * @param orgx X position for the robot centric origin
     * @param orgy Y position for the robot centric origin
     * @param localizer
     * @param modules
     */
    public Drivetrain(double orgx, double orgy, Localizer localizer, SwerveModule[] modules) {
        this.orgx = orgx;
        this.orgy = orgy;
        this.localizer = localizer;
        this.modules = modules;
    }

    public void drive(Gamepad gamepad) {
        for (SwerveModule module : modules) {
            module.fwdPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2));

            double strafeAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
            // Perpendicular angle to robot origin
            double turnAngle = (Math.PI / 2) + Math.atan2(module.y + orgy, module.y + orgy);

            module.setTargetAngle(Util.clampAngle(strafeAngle + turnAngle));
        }
    }
}
