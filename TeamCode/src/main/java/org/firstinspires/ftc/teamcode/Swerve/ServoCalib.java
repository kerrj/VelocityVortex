package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.Constants;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 10/21/2016.
 */
@TeleOp
public class ServoCalib extends Robot {
    AbsoluteEncoder lfe;
    AbsoluteEncoder rfe;
    AbsoluteEncoder lbe;
    AbsoluteEncoder rbe;
    @Override
    public void init() {
        super.init();

        lfe=new AbsoluteEncoder(0,lfa);
        lbe=new AbsoluteEncoder(0,lba);
        rfe=new AbsoluteEncoder(0,rfa);
        rbe=new AbsoluteEncoder(0,rba);

    }

    @Override
    public void loop() {
        telemetry.addData("lf",Math.toDegrees(lfe.getAngle()));
        telemetry.addData("rf",Math.toDegrees(rfe.getAngle()));
        telemetry.addData("lb",Math.toDegrees(lbe.getAngle()));
        telemetry.addData("rb",Math.toDegrees(rbe.getAngle()));
    }
}
