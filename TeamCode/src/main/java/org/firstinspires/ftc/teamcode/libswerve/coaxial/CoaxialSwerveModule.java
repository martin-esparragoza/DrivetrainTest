package org.firstinspires.ftc.teamcode.libswerve.coaxial;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;

public class CoaxialSwerveModule extends SwerveModule {
    private final DcMotor forward;
    private final CRServo turn;
    private final AnalogInput encoder;

    /**
     *
     * @param x Reversed
     * @param y Reversed
     * @param forward
     * @param turn
     * @param encoder
     */
    public CoaxialSwerveModule(double x, double y, DcMotor forward, CRServo turn, AnalogInput encoder) {
        super(x, y);

        forward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.forward = forward;
        this.turn = turn;
        this.encoder = encoder;
    }

    @Override
    public void setPowers(double turnPower, double forwardPower) {
        forward.setPower(forwardPower);
        turn.setPower(turnPower);
    }

    @Override
    public double getAngle() {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }
}
