package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 9/14/2016.
 * @author duncan
 */

public class SwerveDrive {
        double pivotX, pivotY;
        DcMotor front, back;
        Servo frontleft, frontright, backleft, backright;
        //Add values when swerve is done and also look at what absolute encoder is for
        Double WHEEL_BASE_WIDTH;//initialized in constructor
        Double WHEEL_BASE_LENGTH;
        SwerveModule[] modules;

        /**
         * Custom constructor for current robot.
         */
        public SwerveDrive(AnalogInput frontLeft,AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                            DcMotor front,DcMotor back,
                            Servo frontleftServo,Servo frontRightServo,Servo backLeftServo,Servo backRightServo,
                            double robotWidth,double robotLength){
            this.front=front;
            this.back=back;
            this.frontleft=frontleftServo;
            this.frontright=frontRightServo;
            this.backleft=backLeftServo;
            this.backright=backRightServo;
            WHEEL_BASE_LENGTH=robotLength;
            WHEEL_BASE_WIDTH=robotWidth;
            //initialize array of modules
            //array can be any size, as long as the position of each module is specified in its constructor
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(front, frontleft, new AbsoluteEncoder(0,frontLeft), -WHEEL_BASE_WIDTH/2, WHEEL_BASE_LENGTH/2),
                    //front right
                    new SwerveModule(front, frontright, new AbsoluteEncoder(0,frontRight), WHEEL_BASE_WIDTH/2, WHEEL_BASE_LENGTH/2),
                    //back left
                    new SwerveModule(back, backleft, new AbsoluteEncoder(0,backLeft), -WHEEL_BASE_WIDTH/2,-WHEEL_BASE_LENGTH/2),
                    //back right
                    new SwerveModule(back, backright, new AbsoluteEncoder(0,backRight), WHEEL_BASE_WIDTH, -WHEEL_BASE_LENGTH/2)
            };
        }

        /**
         * @param pivotX x coordinate in inches of pivot point relative to center of robot
         * @param pivotY y coordinate in inches of pivot point relative to center of robot
         */
        public void setPivot(double pivotX, double pivotY) {
            this.pivotX = pivotX;
            this.pivotY = pivotY;
        }

        /**
         * Drive with field oriented capability
         * @param translationX relative speed in left/right direction (-1 to 1)
         * @param translationY relative speed in forward/reverse direction (-1 to 1)
         * @param rotation relative rate of rotation around pivot point (-1 to 1) positive is clockwise
         * @param heading offset in heading in radians (used for field oriented control)
         */
        private void driveWithOrient(double translationX, double translationY, double rotation, double heading,double powerScale) {
            Vector[] vects = new Vector[modules.length];
            Vector transVect = new Vector(translationX, translationY),
                    pivotVect = new Vector(pivotX, pivotY);

            //if there is only one module ignore rotation
            if (modules.length < 2)
                for (SwerveModule module : modules)
                    module.set(transVect.getAngle(), Math.min(1, transVect.getMagnitude())); //cap magnitude at 1

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

            double power;
            for (int i = 0; i < modules.length; i++) {
                power = vects[i].getMagnitude() / maxPower; //scale down by the largest power that exceeds 100%
                if (power > .05) {
                    modules[i].set(vects[i].getAngle()-Math.PI/2, power*powerScale);
                } else {
                    modules[i].rest();
                }
            }
        }

        /**
         * Regular robot oriented control.
         * @param translationX relative speed in left/right direction (-1 to 1)
         * @param translationY relative speed in forward/reverse direction (-1 to 1)
         * @param rotation relative rate of rotation around pivot point (-1 to 1) positive is clockwise
         */
        public void driveNormal(double translationX, double translationY, double rotation,double powerScale) {
            driveWithOrient(translationX, translationY, rotation, 0,powerScale);
        }

        public void enable() {
            for (SwerveModule module : modules) module.enable();
        }

        public void disable() {
            for (SwerveModule module : modules) module.disable();
        }

        /**
         * Method called every loop iteration, handle all pid control here
         */
        public void update(boolean waitForServos){
            for (SwerveModule module:modules){
                module.update(waitForServos);
            }
        }

    }


