package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libswerve.Drivetrain;
import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.Vector2;
import org.firstinspires.ftc.teamcode.libswerve.coaxial.CoaxialSwerveModule;
import org.firstinspires.ftc.teamcode.modules.MyCoaxialSwerveModule;

import java.util.Set;

@TeleOp(name = "Coax Teleop", group = "teleop")
public class CoaxTeleop extends OpMode {
    private Drivetrain drivetrain;

    @Override
    public void init() {
        SwerveModule m1 = new MyCoaxialSwerveModule(-1, 1,
            hardwareMap.get(DcMotor.class, "FrontLeftM"),
            hardwareMap.get(CRServo.class, "FrontLeftS"),
            hardwareMap.get(AnalogInput.class, "FrontLeftE"));
        SwerveModule m2 = new MyCoaxialSwerveModule(1, 1,
            hardwareMap.get(DcMotor.class, "FrontRightM"),
            hardwareMap.get(CRServo.class, "FrontLeftS"),
            hardwareMap.get(AnalogInput.class, "FrontLeftE"));
        SwerveModule m3 = new MyCoaxialSwerveModule(1, -1,
            hardwareMap.get(DcMotor.class, "BackRight"),
            hardwareMap.get(CRServo.class, "BackRightS"),
            hardwareMap.get(AnalogInput.class, "BackRightE"));
        SwerveModule m4 = new MyCoaxialSwerveModule(-1, -1,
            hardwareMap.get(DcMotor.class, "BackLeftM"),
            hardwareMap.get(CRServo.class, "BackLeftS"),
            hardwareMap.get(AnalogInput.class, "BackLeftE"));

        drivetrain = new Drivetrain(0.8, null, new SwerveModule[]{m1, m2, m3, m4}) {
            @Override
            public void drive(Gamepad gamepad) {
                setPower(new Vector2(gamepad.left_stick_x, gamepad.left_stick_y), gamepad.right_stick_x);
            }
        };
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1);
        drivetrain.update();
    }
}
