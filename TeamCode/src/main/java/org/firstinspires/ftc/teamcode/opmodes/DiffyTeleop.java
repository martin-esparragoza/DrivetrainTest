package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.differential.SwerveModule;
import org.firstinspires.ftc.teamcode.modules.MySwerveModule;

@TeleOp(name = "Diffy TeleOp", group = "teleop")
public class DiffyTeleop extends OpMode {
    private SwerveModule m1 = null, m2 = null;

    @Override
    public void init() {
        // Two diffy swerve modules
        m1 = new MySwerveModule(
            hardwareMap.get(DcMotor.class, "dswerve_m0_0"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_0"),
            hardwareMap.get(AnalogInput.class, "dswerve_e0_0")
        );
        m2 = new MySwerveModule(
            hardwareMap.get(DcMotor.class, "dswerve_m0_1"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_1"),
            hardwareMap.get(AnalogInput.class, "dswerve_e0_1")
        );
    }

    @Override
    public void loop() {
    }
}
