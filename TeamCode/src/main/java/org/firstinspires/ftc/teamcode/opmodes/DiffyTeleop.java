package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libswerve.Drivetrain;
import org.firstinspires.ftc.teamcode.libswerve.Localizer;
import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.Vector2;
import org.firstinspires.ftc.teamcode.modules.MySwerveModule;

import java.util.Vector;

class MyDrivetrain extends Drivetrain {
    public static final double speed = 0.25;

    public MyDrivetrain(double orgx, double orgy, Localizer localizer, SwerveModule[] modules, double maxVelocity, double maxTurnSpeed) {
        super(orgx, orgy, localizer, modules, maxVelocity, maxTurnSpeed);
    }

    @Override
    /**
     * Gamepad left stick: strafing<br>
     * Gamepad right stick: turning<br>
     * <b>DOES NOT UPDATE THE SWERVE MODULES USE <code>Drivetrain.update()</code></b>
     * @param gamepad
     */
    public void setModules(Gamepad gamepad) {
        // Distance, use both gamepads to get the power (this is so it doesn't have to be calculated again)
        for (SwerveModule module : modules) {
            /*double strafeAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x);
            // Perpendicular angle to robot origin
            double turnAngle =
                ((Math.PI / 2) + Math.atan2(module.y - orgy, module.x - orgx)) *
                Math.signum(gamepad.right_stick_x) *
                Math.max(1.0 - leftmag, 0.5 * rightmag); // If both left and right are used turn at half speed in order to also move

            module.setTargetAngle(strafeAngle + turnAngle);*/

            Vector2 strafeVector = new Vector2(gamepad.left_stick_x, gamepad.left_stick_y);
            double strafeVectorMag = strafeVector.mag(); // Store so we don't have to calculate it multiple times
            module.fwdPower = Math.min(strafeVectorMag + Math.abs(gamepad.right_stick_x), 1) * speed;
            Vector2 turnVector = new Vector2(module.y + orgy, -(module.x + orgx));
            //turnVector.mul(Math.signum(gamepad.right_stick_x) * Math.min(Math.abs(gamepad.right_stick_x), 0.5 * strafeVectorMag));
            turnVector.mul((gamepad.right_stick_x + (1.0 - strafeVectorMag)) / 2);
            //turnVector.mul(gamepad.right_stick_x);
            Vector2 finalVector = Vector2.add(turnVector, strafeVector);

            //module.fwdPower = finalVector.mag();

            module.setTargetAngle(Math.atan2(finalVector.y, finalVector.x));
        }
    }
}

@TeleOp(name = "Diffy TeleOp", group = "teleop")
public class DiffyTeleop extends OpMode {
    private SwerveModule m1 = null, m2 = null;
    private Drivetrain drive;

    @Override
    public void init() {
        // Two diffy swerve modules
        m1 = new MySwerveModule(
            4, 0,
            hardwareMap.get(DcMotor.class, "dswerve_m0_0"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_0")
        );
        m2 = new MySwerveModule(
            -4, 0,
            hardwareMap.get(DcMotor.class, "dswerve_m0_1"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_1")
        );
        drive = new MyDrivetrain(0, 0, null, new SwerveModule[]{m1, m2}, 0.1, 0.1);
    }

    @Override
    public void loop() {
        drive.setModules(gamepad1);
        drive.update();
    }
}