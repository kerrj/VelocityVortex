package org.firstinspires.ftc.teamcode.Swerve;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.FTCVuforia;

/**
 * Created by Justin on 10/7/2016.
 */
@Autonomous
public class RedSwerveVuforiaAuto extends OpMode {

    private static final double WIDTH=16;
    private static final double LENGTH=16;

    private enum RobotState{//list states here
        MoveAwayFromWall
    }

    private FTCSwerve swerveDrive;
    private RobotState robotState=RobotState.MoveAwayFromWall;//initialize start state here

    private FTCVuforia vuforia;


    @Override
    public void init() {
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.initVuforia();
        vuforia.addTrackables("FTC_2016-17.xml");
        swerveDrive=new FTCSwerve(hardwareMap.analogInput.get("frontLeftEncoder"),hardwareMap.analogInput.get("frontRightEncoder"),
                                  hardwareMap.analogInput.get("backLeftEncoder"),hardwareMap.analogInput.get("backRightEncoder"),
                                  hardwareMap.dcMotor.get("front"),hardwareMap.dcMotor.get("back"),
                                  hardwareMap.servo.get("frontLeftServo"),hardwareMap.servo.get("frontRightServo"),
                                  hardwareMap.servo.get("backLeftServo"),hardwareMap.servo.get("backRightServo"),WIDTH,LENGTH);
        hardwareMap.dcMotor.get("front").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.dcMotor.get("back").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        switch(robotState){
            case MoveAwayFromWall:
                swerveDrive.resetPosition();
                if(swerveDrive.getInchesTravelled()<5) {
                    swerveDrive.driveTowards(new Vector(0, 1), 1);
                }else{

                }
                break;
        }
        swerveDrive.update();

    }
}
