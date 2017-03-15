package org.firstinspires.ftc.teamcode.VelocityVortex.SuperAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;
/**
 * Created by Justin on 3/7/2017.
 */
@Autonomous
public class BlueShootPark extends Robot {

    enum RobotState{Shoot, Park}

    RobotState state = RobotState.Shoot;



    public void init() { initAutonomous(); }

    @Override
    public void loop() {
        super.loop();

        switch (state) {
            case Shoot:
                driveWithEncoders(0, 1, 0, .5, 56);
                if (driveWithEncoders(0, 1, 0, .5, 56)) {
                    state = RobotState.Park;
                }
                break;
            case Park:
                swerveDrive.drive(0,0,0,0);
                break;

        }
    }

    public void stop() {
        super.stop();
    }
}
