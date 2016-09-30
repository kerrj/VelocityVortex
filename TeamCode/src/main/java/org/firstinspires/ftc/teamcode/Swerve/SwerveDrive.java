package org.firstinspires.ftc.teamcode.Swerve;

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
        Double WHEEL_BASE_WIDTH=16.0;
        Double WHEEL_BASE_LENGTH=16.0;
        SwerveModule[] modules;

        /**
         * Custom constructor for current robot.
         */
        public SwerveDrive() {
            //initialize array of modules
            //array can be any size, as long as the position of each module is specified in its constructor
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(front, frontleft, new AbsoluteEncoder(0), WHEEL_BASE_WIDTH/2, WHEEL_BASE_LENGTH/2),
                    //front right
                    new SwerveModule(front, frontright, new AbsoluteEncoder(0), WHEEL_BASE_WIDTH/2, WHEEL_BASE_LENGTH/2),
                    //back left
                    new SwerveModule(back, backleft, new AbsoluteEncoder(0), WHEEL_BASE_WIDTH/2, WHEEL_BASE_LENGTH/2),
                    //back right
                    new SwerveModule(back, backright, new AbsoluteEncoder(0), WHEEL_BASE_WIDTH, WHEEL_BASE_LENGTH/2)
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
        private void driveWithOrient(double translationX, double translationY, double rotation, double heading) {
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
                    modules[i].set(vects[i].getAngle()-Math.PI/2, power);
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
        public void driveNormal(double translationX, double translationY, double rotation) {
            driveWithOrient(translationX, translationY, rotation, 0);
        }

        public void enable() {
            for (SwerveModule module : modules) module.enable();
        }

        public void disable() {
            for (SwerveModule module : modules) module.disable();
        }

        /**
         * 2D Mathematical Vector
         */
        private class Vector {
            double x = 0, y = 0;

            public Vector(double x, double y) {
                this.x = x;
                this.y = y;
            }

            public double getAngle() {
                return Math.atan2(y, x);
            }

            public double getMagnitude() {
                return Math.hypot(x, y);
            }

            public void scale(double scalar) {
                x *= scalar;
                y *= scalar;
            }

            public void add(Vector v) {
                x += v.x;
                y += v.y;
            }

            public void subtract(Vector v) {
                x -= v.x;
                y -= v.y;
            }

            public void makePerpendicular() {
                double temp = x;
                x = y;
                y = -temp;
            }
        }
    }


