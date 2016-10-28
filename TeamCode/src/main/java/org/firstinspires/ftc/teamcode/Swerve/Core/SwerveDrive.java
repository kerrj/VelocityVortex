package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 9/14/2016.
 * @author duncan
 */

public class SwerveDrive {
    DcMotor left, right;
    //Add values when swerve is done and also look at what absolute encoder is for
    SwerveModule[] modules;
    double[] angles={0,0,0,0};
    double[] powers={0,0,0,0};
    boolean[] atPositions={false,false,false,false};
//    private double leftTargetPower=0;
//    private double rightTargetPower=0;

    /**
     * Custom constructor for current robot.
     */
    public SwerveDrive(AnalogInput frontLeft,AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                        DcMotor lf,DcMotor rf,DcMotor lb,DcMotor rb,
                        Servo frontleftServo,Servo frontRightServo,Servo backLeftServo,Servo backRightServo,double width,double length){
        this.left=left;
        this.right=right;
        //initialize array of modules
        //array can be any size, as long as the position of each module is specified in its constructor
        modules = new SwerveModule[] {
                //front left
                new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
                //front right
                new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
                //back left
                new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
                //back right
                new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
        };
    }

    public void driveWithOrient(double translationX, double translationY, double rotation, double heading, double powerScale) {
        double pivotX=0;
        double pivotY=0;
        Vector[] vects = new Vector[modules.length];
        Vector transVect = new Vector(translationX, translationY),
                pivotVect = new Vector(pivotX, pivotY);

        double maxDist = 0;
        for (int i = 0; i < modules.length; i++) {
            vects[i] = new Vector(modules[i].positionX, modules[i].positionY);
            vects[i].subtract(pivotVect); //calculate module's position relative to pivot point
            maxDist = Math.max(maxDist, vects[i].getMagnitude()); //find farthest distance from pivot
        }

        double maxPower = 1;
        for (int i = 0; i < modules.length; i++) {
            //rotation motion created by driving each module perpendicular to
            //the vector from the pivot point
            vects[i].makePerpendicular();
            //scale by relative rate and normalize to the farthest module
            //i.e. the farthest module drives with power equal to 'rotation' variable
            vects[i].scale(rotation / maxDist);
            vects[i].add(transVect);
            //calculate largest power assigned to modules
            //if any exceed 100%, all must be scale down
            maxPower = Math.max(maxPower, vects[i].getMagnitude());
        }
        angles[0]=vects[0].getAngle()+Math.PI;
        angles[1]=vects[1].getAngle()+Math.PI;
        angles[2]=vects[2].getAngle()+Math.PI;
        angles[3]=vects[3].getAngle();

        powers[0]=(vects[0].getMagnitude() / maxPower)*powerScale;
        powers[1]=(vects[1].getMagnitude() / maxPower)*powerScale;
        powers[2]=(vects[2].getMagnitude() / maxPower)*powerScale;
        powers[3]=(vects[2].getMagnitude() / maxPower)*powerScale;

//        modules[0].set(vects[0].getAngle()+Math.PI,  (vects[0].getMagnitude() / maxPower)*powerScale);
//        modules[2].set(vects[2].getAngle()+Math.PI, (vects[2].getMagnitude() / maxPower)*powerScale);
//        modules[1].set(vects[1].getAngle()+Math.PI, (vects[1].getMagnitude() / maxPower)*powerScale);
//        modules[3].set(vects[3].getAngle(), (vects[2].getMagnitude() / maxPower)*powerScale);
    }








//    public void translate(Vector direction,double power){
//        leftTargetPower=power;
//        rightTargetPower=power;
//
//        //left side
//        modules[0].set(direction.getAngle());
//        modules[2].set(direction.getAngle()+Math.PI);//intentionally reversed
//        if(Math.abs(modules[0].getDelta())>Math.PI/2){
//            leftTargetPower*=-1;
//            modules[0].set(direction.getAngle()+Math.PI);
//            modules[2].set(direction.getAngle());//intentionally reversed
//        }
//
//        //right side
//        modules[1].set(direction.getAngle());
//        modules[3].set(direction.getAngle());
//        if(Math.abs(modules[1].getDelta())>Math.PI/2){
//            rightTargetPower*=-1;
//            modules[1].set(direction.getAngle()+Math.PI);
//            modules[3].set(direction.getAngle()+Math.PI);
//        }
//    }
//
//    public void rotate(double power){
//        leftTargetPower=-power;//intentionally negative
//        rightTargetPower=power;
//
//        //left side
//        modules[0].set(Math.PI/4);
//        modules[2].set(3*Math.PI/4);
//        if(Math.abs(modules[0].getDelta())>Math.PI/2){
//            leftTargetPower*=-1;
//            modules[0].set(Math.PI/4+Math.PI);
//            modules[2].set(3*Math.PI/4+Math.PI);
//        }
//
//        //right side
//        modules[1].set(7*Math.PI/4);
//        modules[3].set(5*Math.PI/4);
//        if(Math.abs(modules[1].getDelta())>Math.PI/2){
//            rightTargetPower*=-1;
//            modules[1].set(7*Math.PI/4+Math.PI);
//            modules[3].set(5*Math.PI/4+Math.PI);
//        }
//    }

    /**
     * Method called every loop iteration
     */
    public void update(boolean waitForServos,double threshold){
        if(!waitForServos) {
            for(int i=0;i<modules.length;i++){
                SwerveModule module=modules[i];
                module.set(angles[i],powers[i]);
                module.update();
            }
        }else{
            boolean atPosition=true;
            for(int i=0;i<modules.length;i++){
                SwerveModule module=modules[i];
                module.set(angles[i],0);//necessary
                if (Math.abs(module.getDelta())>Math.toRadians(threshold)){
                    atPosition=false;
                    atPositions[i]=false;
                }else{
                    atPositions[i]=true;
                }
            }


            if(atPosition){//if all are at the correct position
                for(int i=0;i<modules.length;i++){
                    SwerveModule module=modules[i];
                    module.set(angles[i],powers[i]);
                    module.update();
                }
            }else{
                for(int i=0;i<modules.length;i++){
                    SwerveModule module=modules[i];
                    if(atPositions[i]){
                        module.set(angles[i],0);
                    }else{
                        if(module.getDirection()== SwerveModule.ModuleDirection.clockwise){
                            if(i==3){
                                module.setPower(-.1);//intentionally different
                            }else {
                                module.setPower(.1);//adjust signs on these depending on motor direction
                            }
                        }else{
                            if(i==3){
                                module.setPower(.1);//intentionally different
                            }else {
                                module.setPower(-.1);
                            }
                        }

                    }
                    module.update();
                }
            }
        }
    }

    public void stop(){
        for(int i=0;i<modules.length;i++){
            SwerveModule module=modules[i];
            powers[i]=0;
            module.stop();
        }
    }

    public void lockWheels(){
//        leftTargetPower=0;
//        rightTargetPower=0;
        modules[0].set(3*Math.PI/4,0);
        modules[1].set(Math.PI/4,0);
        modules[2].set(Math.PI/4,0);
        modules[3].set(3*Math.PI/4,0);
        for(int i=0;i<modules.length;i++){
            SwerveModule module=modules[i];
            powers[i]=0;
            module.stop();
        }
    }

}


