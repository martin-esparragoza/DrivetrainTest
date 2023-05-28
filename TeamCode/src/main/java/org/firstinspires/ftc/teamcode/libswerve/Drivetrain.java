package org.firstinspires.ftc.teamcode.libswerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Drivetrain {
    public final Localizer localizer;
    protected final SwerveModule[] modules;
    private final ElapsedTime timer = new ElapsedTime();
    public double speed = 0;

    /**
     *
     * @param localizer
     * @param modules
     */
    public Drivetrain(double speed, Localizer localizer, SwerveModule[] modules) {
        this.speed = speed;
        this.localizer = localizer;
        this.modules = modules;
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

    public void brake() {
        for (SwerveModule module : modules) {
            module.fwdPower = 0;
        }
    }

    public void render(FtcDashboard dash) {
        TelemetryPacket p = new TelemetryPacket();
        p.fieldOverlay()
            .setFill("green")
            .fillRect(0, 0, 100, 100);
        dash.sendTelemetryPacket(p);
    }
}
