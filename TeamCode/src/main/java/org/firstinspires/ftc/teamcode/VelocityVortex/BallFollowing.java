package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CameraStuff.BallTracking;
import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;
import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
@Disabled
public class BallFollowing extends Robot {
    BallTracking ballTracking;
    private enum State{Driving,Tracking}
    private State state=State.Driving;

    @Override
    public void init() {
        initAutonomous();
        vuforia.pauseVuforia();
        ballTracking=new BallTracking(this,Side.BLUE);
        ballTracking.startCamera();
    }

    public void init_loop(){
        if(swerveDrive==null){
            swerveDrive=new FTCSwerve(lfa, rfa, lba, rba, lfm, rfm, lbm, rbm, lf, rf, lb, rb, 14, 14);
        }
        swerveDrive.drive(-1, 0, 0, 0);
        swerveDrive.update(true, 15, false);
    }


    Vector direction=new Vector(0, 1);
    boolean wheelIn=true,wheelsRotated=false,doneShooting=true;
    @Override
    public void loop() {
        super.loop();
        if(swerveDrive==null){
            swerveDrive=new FTCSwerve(lfa, rfa, lba, rba, lfm, rfm, lbm, rbm, lf, rf, lb, rb, 14, 14);
        }

        switch(state){
            case Driving:
                Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
                if(gamepad1.left_bumper){
                    if (d.getMagnitude() > .02 || Math.abs(gamepad1.right_stick_x) > .02) {
                        direction = d;
                        //                swerveDrive.drive(Math.cos(angleBetween+inputAngle)*direction.getMagnitude(), Math.sin(angleBetween-inputAngle)*direction.getMagnitude(), gamepad1.right_stick_x/1.5, .2);
                        swerveDrive.drive(direction.x,direction.y,gamepad1.right_stick_x/3,.2);
                        if(Math.abs(gamepad1.right_stick_x)>.02&&d.getMagnitude()<.02){
                            wheelsRotated=true;
                        }else{
                            wheelsRotated=false;
                        }
                    } else if(wheelsRotated) {
                        swerveDrive.drive(0,0,1,0);
                    } else {
                        //                swerveDrive.drive(Math.cos(angleBetween+inputAngle)*direction.getMagnitude(), Math.sin(angleBetween-inputAngle)*direction.getMagnitude(), gamepad1.right_stick_x/1.5, 0);
                        swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/3, 0);
                    }
                }else {
                    if (d.getMagnitude() > .02 || Math.abs(gamepad1.right_stick_x) > .02) {
                        direction = d;
                        //                swerveDrive.drive(Math.cos(angleBetween-inputAngle)*direction.getMagnitude(), Math.sin(angleBetween-inputAngle)*direction.getMagnitude(), gamepad1.right_stick_x/1.5, .5);
                        swerveDrive.drive(direction.x,direction.y,gamepad1.right_stick_x/3,1);
                        if(Math.abs(gamepad1.right_stick_x)>.02&&d.getMagnitude()<.02){
                            wheelsRotated=true;
                        }else{
                            wheelsRotated=false;
                        }
                    } else if(wheelsRotated){
                        swerveDrive.drive(0,0,1,0);
                    }else {
                        //                swerveDrive.drive(Math.cos(angleBetween+inputAngle)*direction.getMagnitude(), Math.sin(angleBetween-inputAngle)*direction.getMagnitude(), gamepad1.right_stick_x/1.5, 0);
                        swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/3, 0);
                    }
                }

                swerveDrive.update(true,40,false);
                if(gamepad1.a){
                    state=State.Tracking;
                }
                break;

            case Tracking:
                ballTracking.scoreBallz();
                if(gamepad1.b){
                    ballTracking.trackingState= BallTracking.TrackingState.Searching;
                    state=State.Driving;
                }
                break;
        }

        ballTracking.scoreBallz();
        swerveDrive.update(true,30,true);
    }

    public void stop(){
        super.stop();
        thread.kill();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
