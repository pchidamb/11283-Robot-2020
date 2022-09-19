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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Competition_11283: Teleop 2020", group="Teleop11283")
//@Disabled
public class TeleopTank_Iterative_11283 extends OpMode {

    /* Declare OpMode members. */
    Hardware_11283 robot = new Hardware_11283(); // use the class created to define a Pushbot's hardware

     // Code to run ONCE when the driver hits INIT

//    double speedFactor = .60;
//    double armPower = .4;
//    double liftPower = .75;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("gamepad1, button y =", "start gatherer");
        telemetry.addData("gamepad1, button b =", "stop gatherer");
        telemetry.addData("gamepad1, button a =", "reverse gatherer");
        telemetry.addData("gamepad2, dpad up and down =", "Lift up and down");
        telemetry.addData("gamepad2, buttons a and y =", "grabber closed and open");
        telemetry.addData("gamepad2, buttons x and b =", "wobble motor up and down");
        telemetry.addData("gamepad1, button a =", "start and stop gatherer");
    }
     //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
    }

     // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }
     // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        //declare variables to use in the loop
        double leftFront_Speed;
        double leftRear_Speed;
        double rightFront_Speed;
        double rightRear_Speed;
        double motorPower = 0;
        boolean changed = false;
//        double lfs;
//        double rfs;

//add this vairable.  "CC" stands for controller coefficient and is used with mechanum wheels so the x and y sticks don't add up to more than 1 for a pair of motors (this would be bad since the motor value is limited to 1.  The motors moving in the opposing direction would not be limited
        double CC = .5;
//This code is for mecanum wheels mounted out the sides of robot (team 11283)
        leftFront_Speed = (gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        rightFront_Speed = (-gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        leftRear_Speed = (gamepad1.left_stick_y*(CC) + gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        rightRear_Speed = (-gamepad1.left_stick_y*(CC) +gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));

        // (note: The joystick goes negative when pushed forwards, so negate it)
        robot.Motor_leftFront.setPower(leftFront_Speed);
        robot.Motor_leftRear.setPower(leftRear_Speed);
        robot.Motor_rightFront.setPower(rightFront_Speed);
        robot.Motor_rightRear.setPower(rightRear_Speed);

//Use these examples to move the grabber hand based on game pad buttons
//      This sets a value for the servo. "robot." means that the variable is coming from the Hardware_11283 class
        /*if (gamepad2.dpad_up) {
            robot.Motor_lift.setPower(-1);
            while (robot.Limit_lift_up.getState() == true) {
                //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
                telemetry.addData("Lift motor touch_Up status", robot.Limit_lift_up.getState());
                telemetry.update();
            }
            robot.Motor_lift.setPower(0);
        };

        if (gamepad2.dpad_down) {
            robot.Motor_lift.setPower(1);
            while (robot.Limit_lift_down.getState() == true) {
                //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
                telemetry.addData("Lift motor touch_Down status", robot.Limit_lift_down.getState());
                telemetry.update();
            }
            robot.Motor_lift.setPower(0);
        };*/
        if (gamepad2.dpad_down && (robot.Limit_lift_down.getState() == true)) {
            robot.Motor_lift.setPower(1);

        }
        else {
            if (gamepad2.dpad_up && (robot.Limit_lift_up.getState() == true) ) {
                robot.Motor_lift.setPower(-1);
            }
            else {
                robot.Motor_lift.setPower(0);
            }
        }

        if (gamepad2.b) {
            robot.Servo_grabber.setPosition(robot.servo_grabber_closed);

        };

        if (gamepad2.x){
           robot.Servo_grabber.setPosition(robot.servo_grabber_open);

       };

     /*   if (gamepad2.b) {
            robot.Motor_wobble.setPower(1);
            while (robot.Limit_wobble_down.getState() == true) {
                //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
                telemetry.addData("Wobble touch down status", robot.Limit_wobble_down.getState());
                telemetry.update();
            }
            robot.Motor_wobble.setPower(0);
        };

        if (gamepad2.x) {
            robot.Motor_wobble.setPower(-1);
            while (robot.Limit_wobble_up.getState() == true) {
                //       keep running lift motor up until it hits the upper limit switch, a limit switch turns false when pressed
                telemetry.addData("Wobble touch up status", robot.Limit_wobble_up.getState());
                telemetry.update();
            }
            robot.Motor_wobble.setPower(0);
        };*/
     // x is up, bis down
        telemetry.addData("Wobble down status", robot.Limit_wobble_down.getState());
        telemetry.addData("Wobble up status", robot.Limit_wobble_up.getState());
      //  telemetry.update();

        if (gamepad2.a && (robot.Limit_wobble_down.getState() == true)) {
            robot.Motor_wobble.setPower(.1);

        }
        else {
            if (gamepad2.y && (robot.Limit_wobble_up.getState() == true) ) {
                robot.Motor_wobble.setPower(-.5);
            }
            else {
                robot.Motor_wobble.setPower(0);
            }
        }

//this section gives the driver 3 buttons to turn on, off and reverse the gatherer and comment out the code to use one button that turns it on and off
     if(gamepad1.y) {
         robot.Motor_gatherer.setPower(.7);
         robot.Motor_gatherer_lift.setPower(1);
     }
     else if (gamepad1.a){
         robot.Motor_gatherer.setPower(-.7);
         robot.Motor_gatherer_lift.setPower(-1);
     }
     else if(gamepad1.b) {
         robot.Motor_gatherer.setPower(0);
         robot.Motor_gatherer_lift.setPower(0);
     }
     //This code uses one button to turn on and turn off motors
//        if(gamepad1.a && !changed) {
//            if(robot.Motor_gatherer.getPower() == 0)
//            { robot.Motor_gatherer.setPower(.7);
//            robot.Motor_gatherer_lift.setPower(1);
//                }
//            else
//                robot.Motor_gatherer.setPower(0);
//            robot.Motor_gatherer.setPower(.7);
//            robot.Motor_gatherer_lift.setPower(1);
//            changed = true;
//        } else if(!gamepad1.a) changed = false;


        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();

 //use the example below to set up the Motor_gatherer and Motor_lift
        /*  if (gamepad2.y) {
            if (robot.limit_robot_lift_top.getState())
                robot.robot_lift_motor.setPower(1);
            else
                robot.robot_lift_motor.setPower(0);
        }
        else {
            if (gamepad2.a) {
                if (robot.limit_robot_lift_bottom.getState())
                    robot.robot_lift_motor.setPower(-1);
                else
                    robot.robot_lift_motor.setPower(0);
            }
            else {
                robot.robot_lift_motor.setPower(0);
            }
        }

        if (gamepad2.x) {
            if (robot.limit_robot_hook_top.getState())
                robot.hook_lift_motor.setPower(1);
            else
                robot.hook_lift_motor.setPower(0);
        }
        else {
            if (gamepad2.b) {
                if (robot.limit_robot_hook_bottom.getState())
                    robot.hook_lift_motor.setPower(-1);
                else
                    robot.hook_lift_motor.setPower(0);
            }
            else {
                robot.hook_lift_motor.setPower(0);
            }
        } */

//example of checking if a variable is equal to a value and taking an action
//        if (gst == 0) {
//            leftFront_Speed = -leftFront_Speed;
//        };
//example of setting a variable from gamepad buttons
        // if (gamepad2.x) {
        //    lfs = 0.0;
        //};


 //Use this code to test color sensors in teleop
       //this part is to get HSV values out of the Rev robotics color sensor
        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues_RF[] = {0F, 0F, 0F};
//        float hsvValues_LF[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
//        final float values_RF[] = hsvValues_RF;
//        final float values_LF[] = hsvValues_LF;
//        // sometimes it helps to multiply the raw RGB values with a scale factor
//        // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//        Color.RGBToHSV((int) (robot.sensorColorRightFront.red() * SCALE_FACTOR),
//                (int) (robot.sensorColorRightFront.green() * SCALE_FACTOR),
//                (int) (robot.sensorColorRightFront.blue() * SCALE_FACTOR),hsvValues_RF);



        // telemetry.addData("Alpha_LF", robot.sensorColorLeftFront.alpha());
        //telemetry.addData("Red_LF  ", robot.sensorColorLeftFront.red());
        //telemetry.addData("Green_LF", robot.sensorColorLeftFront.green());
        //telemetry.addData("Blue_LF ", robot.sensorColorLeftFront.blue());
        //telemetry.addData("Hue_LF", hsvValues_LF[0]);

        //telemetry.addData("Distance_RF (inch)",
         //       String.format(Locale.US, "%.02f", robot.sensorDistanceRightFront.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Distance_LF (inch)",
           //     String.format(Locale.US, "%.02f", robot.sensorDistanceLeftFront.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Distance_RS (inch)",
             //   String.format(Locale.US, "%.02f", robot.sensorDistanceRightSide.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Distance_LS (inch)",
               // String.format(Locale.US, "%.02f", robot.sensorDistanceLeftSide.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Distance_Up (inch)",
                //String.format(Locale.US, "%.02f", robot.sensorDistanceUp.getDistance(DistanceUnit.INCH)));


        //Rev 2meter distance sensor
        //telemetry.addData("range2m", String.format("%.01f in", robot.sensorRange2m.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("raw ultrasonic", robot.sensorRangeUltra.rawUltrasonic());
        //telemetry.addData("raw optical ultrasonic ultrasonic", robot.sensorRangeUltra.rawOptical());
        //telemetry.addData("cm optical utrasonic", "%.2f cm", robot.sensorRangeUltra.cmOptical());
        //telemetry.addData("cm optical utrasonic", "%.2f cm", robot.sensorRangeUltra.cmUltrasonic());
        //telemetry.addData("cm ultrasonic", "%.2f cm", robot.sensorRangeUltra.getDistance(DistanceUnit.INCH));

        //telemetry.update();

//        if (robot.lift_Motor_Touch_Down.getState() == true) {
//            telemetry.addData("lift motor down", "Is Not Pressed");
//        } else {
//            telemetry.addData("lift motor down", "is pressed");
//        };
//
//        if (robot.lift_Motor_Touch_Up.getState() == true) {
//            telemetry.addData("lift motor up", "Is Not Pressed");
//        } else {
//            telemetry.addData("lift motor up", "is pressed");
//        };

        // send the info back to driver station using telemetry function.

    }

     // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

    //public void moveRightArm(.5) {

      //  if (power < 0) {
        //    if (robot.right_Touch_In.getState() == false) {
                // pressed, stop arm
          //      power = 0;
            //}

        //};

        //if (power > 0) {
          //  if (robot.right_Touch_Out.getState() == false) {
                // pressed, stop arm
            //    power = 0;
            //}
        //};
       // robot.Right_Servo_Arm.setPower(power);
    //}

//    public void moveLeftArm(double power) {
  //      if (power < 0) {
    //        if (robot.left_Touch_In.getState() == false) {
                // pressed, stop arm
      //          power = 0;
        //    }
        //};

        //if (power > 0) {
          //  if (robot.left_Touch_Out.getState() == false) {
                // pressed, stop arm
            //    power = 0;
            //}

        //};
        //robot.Left_Servo_Arm.setPower(power);
    //}

//    public void moveLiftArm(double power) {
//
//        if (power < 0) {
//            if (robot.lift_Motor_Touch_Down.getState() == false) {
//                // pressed, stop arm
//                power = 0;
//            }
//        };
//        if (power > 0) {
//            if (robot.lift_Motor_Touch_Up.getState() == false) {
//                // pressed, stop arm
//                power = 0;
//            }
//        };
//        robot.LiftMotor.setPower(power);
//    }


}