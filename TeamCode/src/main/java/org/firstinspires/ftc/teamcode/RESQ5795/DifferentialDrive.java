package org.firstinspires.ftc.teamcode.RESQ5795;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Justin on 12/29/2016.
 */
@TeleOp
public class DifferentialDrive extends OpMode {

    DcMotor left,right;

    @Override
    public void init() {
        left=hardwareMap.dcMotor.get("left");
        right=hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.left_stick_y);
    }
}
