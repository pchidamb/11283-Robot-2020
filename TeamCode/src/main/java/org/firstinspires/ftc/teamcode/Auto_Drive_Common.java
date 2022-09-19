/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Pushbot: Auto Common", group="Pushbot")
@Disabled


public abstract class Auto_Drive_Common extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_11283 robot = new Hardware_11283();
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LIFT_COUNTS_PER_INCH = (140 * 6);

    public void lift_up() {
        //code to perform lift up

        robot.Motor_lift.setPower(-1);
        while (opModeIsActive() && robot.Limit_lift_up.getState() == true) {
            //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
            telemetry.addData("Lift motor touch_Up status", robot.Limit_lift_up.getState());
            telemetry.update();
        }
        robot.Motor_lift.setPower(0);
    }

    public void lift_down() {
        //code to perform lift down
        robot.Motor_lift.setPower(.25);
        while (opModeIsActive() && (robot.Limit_lift_down.getState() == true)) {
            //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
            telemetry.addData("Lift motor touch_Down status", robot.Limit_lift_down.getState());
            telemetry.update();
        }
        robot.Motor_lift.setPower(0);
    }

    public void wobble_arm_out() {
        //code to perform wobble arm out
        robot.Motor_wobble.setPower(1);
        while (opModeIsActive() && (robot.Limit_wobble_down.getState() == true)) {
            //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
            telemetry.addData("Wobble touch down status", robot.Limit_wobble_down.getState());
            telemetry.update();
        }
        robot.Motor_wobble.setPower(0);
    }

    public void wobble_arm_in() {
        //code to perform wobble arm in
        robot.Motor_wobble.setPower(-1);
        while (opModeIsActive() && (robot.Limit_wobble_up.getState() == true)){
            //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
            telemetry.addData("Wobble touch up status", robot.Limit_wobble_up.getState());
            telemetry.update();
        }
        robot.Motor_wobble.setPower(0);
    }

    public void grabber_open() {
        //code to perform grabber open
        robot.Servo_grabber.setPosition(robot.servo_grabber_open);
        idle();
        sleep(700);
    }

    public void grabber_closed() {
        //code to perform grabber closed
        robot.Servo_grabber.setPosition(robot.servo_grabber_closed);
        idle();
        sleep(700);
    }


//    public void raiseLiftArm () {
//        runtime.reset();
//        robot.LiftMotor.setPower(1);
//        robot.lift_Motor_Touch_Up.getState();
//        while (opModeIsActive() && (runtime.seconds() < 1)&& (robot.lift_Motor_Touch_Up.getState() == true) ) {
//            //would like to add && (robot.lift_Motor_Touch_Up.getState() == true)
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//            robot.lift_Motor_Touch_Up.getState();
//            telemetry.addData("Lift motor touch_Up status", robot.lift_Motor_Touch_Up.getState());
//            telemetry.addData("Lift motor touch_Down status", robot.lift_Motor_Touch_Down.getState());
//            telemetry.update();
//        }
//        robot.LiftMotor.setPower(0);
//
//        idle();
//        sleep(250);
//    }

//    public void lowerLiftArm () {
//        runtime.reset();
//        robot.LiftMotor.setPower(-1);
//        while (opModeIsActive() && (runtime.seconds() < 2)&&(robot.lift_Motor_Touch_Down.getState() == true)) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//            robot.lift_Motor_Touch_Down.getState();
//            telemetry.addData("Lift motor touch_Down status", robot.lift_Motor_Touch_Down.getState());
//            telemetry.update();
//        }
//        robot.LiftMotor.setPower(0);
//
//        idle();
//        sleep(250);
//
//    }

//    public void closeRightGrabber () {
//        robot.Servo_rightGrabber.setPosition(.75);
//        idle();
//        sleep(250);
//    }
//
//
//    public void openRightGrabber () {
//        robot.Servo_rightGrabber.setPosition(0);
//        idle();
//        sleep(250);
//    }
//    public void closeLeftGrabber () {
//        robot.Servo_leftGrabber.setPosition(.75);
//        idle();
//        sleep(250);
//    }


//    public void openLeftGrabber () {
//        robot.Servo_leftGrabber.setPosition(0);
//        idle();
//        sleep(250);
//    }

        public void DriveForwardUntilBlueLine (double seconds, double power) {

            NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
            final float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            robot.Motor_leftFront.setPower(-power);
            robot.Motor_leftRear.setPower(-power);
            robot.Motor_rightFront.setPower(power);
            robot.Motor_rightRear.setPower(power);
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < seconds) && (colors.blue < .006)&& (colors.green < .012)) {

                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red)
                        .addData("Green", "%.3f", colors.green)
                        .addData("Blue", "%.3f", colors.blue);
                telemetry.update();
                colors = robot.colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
            }
            robot.Motor_rightFront.setPower(0);
            robot.Motor_leftFront.setPower(0);
            robot.Motor_rightRear.setPower(0);
            robot.Motor_leftRear.setPower(0);

            idle();
            sleep(250);
        }

    public void DriveBackwardUntilWhiteLine (double seconds, double power) {

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        robot.Motor_leftFront.setPower(power);
        robot.Motor_leftRear.setPower(power);
        robot.Motor_rightFront.setPower(-power);
        robot.Motor_rightRear.setPower(-power);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < seconds) && (colors.green < .015))  {

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.update();
            colors = robot.colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
        }
        robot.Motor_rightFront.setPower(0);
        robot.Motor_leftFront.setPower(0);
        robot.Motor_rightRear.setPower(0);
        robot.Motor_leftRear.setPower(0);

        idle();
        sleep(250);
    }



//    public void turnWheelsRight90ThenUntilWall (double seconds, double power) {
//
//        double dist = 0;
//
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.5);
//        robot.Servo_rightRear.setPosition(.5);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(power);
//        robot.Motor_rightRear.setPower(power);
//
//        runtime.reset();
//
//       // dist = robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH);
//        while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 5) || (Double.isNaN(dist)))) {
//            //while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//           // telemetry.addData("sensor distance", robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//         //   dist = robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH);
//        }
//        robot.Motor_rightFront.setPower(0);
//        robot.Motor_leftFront.setPower(0);
//        robot.Motor_rightRear.setPower(0);
//        robot.Motor_leftRear.setPower(0);
//
//        idle();
//        sleep(250);
//    }
//
//    public void turnWheelsLRight90ThenMoveUntilWall (double seconds, double power) {
//        double dist = 0;
//
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.5);
//        robot.Servo_rightRear.setPosition(.5);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(power);
//        robot.Motor_rightRear.setPower(power);
//        runtime.reset();
//
//
//      //  dist = robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH);
//        while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//           // telemetry.addData("sensor distance", robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//         //   dist = robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH);
//        }
//
//        robot.Motor_rightFront.setPower(0);
//        robot.Motor_leftFront.setPower(0);
//        robot.Motor_rightRear.setPower(0);
//        robot.Motor_leftRear.setPower(0);
//
//        idle();
//        sleep(250);
//    }
//    public void turnWheelsStraight90ThenMoveToBlock (double seconds, double power) {
//
//        robot.Servo_leftFront.setPosition(1);
//        robot.Servo_leftRear.setPosition(1);
//        robot.Servo_rightFront.setPosition(.05);
//        robot.Servo_rightRear.setPosition(.05);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(-power);
//        robot.Motor_rightRear.setPower(-power);
//
//        runtime.reset();
//        double dist = 0;
//       // dist = robot.sensorDistanceRightFront.getDistance(DistanceUnit.INCH);
//        while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//           // telemetry.addData("sensor distance", robot.sensorDistanceRightFront.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//           // dist = robot.sensorDistanceRightFront.getDistance(DistanceUnit.INCH);
//        }
//        robot.Motor_rightFront.setPower(0);
//        robot.Motor_leftFront.setPower(0);
//        robot.Motor_rightRear.setPower(0);
//        robot.Motor_leftRear.setPower(0);
//
//        idle();
//        sleep(250);
//    }

    public void moveLeft(double seconds, double power)

    {
        robot.Motor_leftFront.setPower(power);
        robot.Motor_leftRear.setPower(-power);
        robot.Motor_rightFront.setPower(power);
        robot.Motor_rightRear.setPower(-power);
        moveRobot(seconds);

        idle();
        sleep(250);
    }

    public void moveRight(double seconds, double power)

    {
        robot.Motor_leftFront.setPower(-power);
        robot.Motor_leftRear.setPower(power);
        robot.Motor_rightFront.setPower(-power);
        robot.Motor_rightRear.setPower(power);
        moveRobot(seconds);

        idle();
        sleep(250);
    }

    public void moveForward(double seconds, double power)

    {
        robot.Motor_leftFront.setPower(-power);
       robot.Motor_leftRear.setPower(-power);
       robot.Motor_rightFront.setPower(power);
      robot.Motor_rightRear.setPower(power);
        moveRobot(seconds);

      idle();
      sleep(250);
    }
    public void moveBack(double seconds, double power)

    {
        robot.Motor_leftFront.setPower(power);
        robot.Motor_leftRear.setPower(power);
        robot.Motor_rightFront.setPower(-power);
        robot.Motor_rightRear.setPower(-power);
        moveRobot(seconds);

        idle();
        sleep(250);
    }

    public void rotateLeft(double seconds, double power)

    {
        robot.Motor_leftFront.setPower(power);
        robot.Motor_leftRear.setPower(power);
        robot.Motor_rightFront.setPower(power);
        robot.Motor_rightRear.setPower(power);
        moveRobot(seconds);

        idle();
        sleep(250);
    }
//    public void turnWheelsRight90ThenMove (double seconds, double power) {
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.485);
//        robot.Servo_rightRear.setPosition(.485);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(power);
//        robot.Motor_rightRear.setPower(power);
//
//        moveRobot(seconds);
//
//        idle();
//        sleep(250);
//
//    }
//
//
//    public void turnWheelsLeft90ThenMove (double seconds, double power) {
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.5);
//        robot.Servo_rightRear.setPosition(.5);
//        sleep(700);
//        robot.Motor_leftFront.setPower(power);
//        robot.Motor_leftRear.setPower(power);
//        robot.Motor_rightFront.setPower(-power);
//        robot.Motor_rightRear.setPower(-power);
//
//        moveRobot(seconds);
//        idle();
//        sleep(250);
//    }
//
//    public void turnWheelsStraightThenMove (double seconds, double power) {
//        robot.Servo_leftFront.setPosition(1);
//        robot.Servo_leftRear.setPosition(1);
//        robot.Servo_rightFront.setPosition(.05);
//        robot.Servo_rightRear.setPosition(.05);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(-power);
//        robot.Motor_rightRear.setPower(-power);
//
//        moveRobot(seconds);
//
//        idle();
//        sleep(250);
//
//
//    }


    public void moveRobot (double seconds) {

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
            telemetry.addData("right front power", robot.Motor_rightFront.getPower());
            telemetry.addData("right rear power", robot.Motor_rightRear.getPower());
            telemetry.addData("left front power", robot.Motor_rightFront.getPower());
            telemetry.addData("left rear power", robot.Motor_rightRear.getPower());

            telemetry.update();
        }
        robot.Motor_rightFront.setPower(0);
        robot.Motor_leftFront.setPower(0);
        robot.Motor_rightRear.setPower(0);
        robot.Motor_leftRear.setPower(0);
    }

//    public void turnWheelsLeft90ThenMoveUnderBridge (double seconds, double power) {
//
//        double dist = 0;
//
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.5);
//        robot.Servo_rightRear.setPosition(.5);
//        sleep(700);
//        robot.Motor_leftFront.setPower(power);
//        robot.Motor_leftRear.setPower(power);
//        robot.Motor_rightFront.setPower(-power);
//        robot.Motor_rightRear.setPower(-power);
//
//        runtime.reset();
//
//        dist = robot.sensorDistanceUp.getDistance(DistanceUnit.INCH);
//        while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//            telemetry.addData("sensor distance", robot.sensorDistanceUp.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//            dist = robot.sensorDistanceUp.getDistance(DistanceUnit.INCH);
//        }
//        robot.Motor_rightFront.setPower(0);
//        robot.Motor_leftFront.setPower(0);
//        robot.Motor_rightRear.setPower(0);
//        robot.Motor_leftRear.setPower(0);
//
//        idle();
//        sleep(250);
//    }

//    public void turnWheelsRight90ThenMoveUnderBridge (double seconds, double power) {
//
//        double dist = 0;
//
//        robot.Servo_leftFront.setPosition(.5);
//        robot.Servo_leftRear.setPosition(.5);
//        robot.Servo_rightFront.setPosition(.485);
//        robot.Servo_rightRear.setPosition(.485);
//        sleep(700);
//        robot.Motor_leftFront.setPower(-power);
//        robot.Motor_leftRear.setPower(-power);
//        robot.Motor_rightFront.setPower(power);
//        robot.Motor_rightRear.setPower(power);
//
//        runtime.reset();
//
//        dist = robot.sensorDistanceUp.getDistance(DistanceUnit.INCH);
//        while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //while (opModeIsActive() && (runtime.seconds() < seconds) && ((dist > 3.5) || (Double.isNaN(dist)))) {
//            //telemetry.addData("RightFront busy", robot.Motor_rightFront.isBusy());
//            telemetry.addData("sensor distance", robot.sensorDistanceUp.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//            dist = robot.sensorDistanceUp.getDistance(DistanceUnit.INCH);
//        }
//        robot.Motor_rightFront.setPower(0);
//        robot.Motor_leftFront.setPower(0);
//        robot.Motor_rightRear.setPower(0);
//        robot.Motor_leftRear.setPower(0);
//
//        idle();
//        sleep(250);
//    }






}
