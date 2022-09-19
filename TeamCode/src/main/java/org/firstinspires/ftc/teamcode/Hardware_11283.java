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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 */


// Set the rotation servo for extended PWM range


public class Hardware_11283
{
     /* Public OpMode members. */

    public DcMotor Motor_leftFront  = null;
    public DcMotor Motor_leftRear    = null;
    public DcMotor Motor_rightFront    = null;
    public DcMotor Motor_rightRear    = null;
    public DcMotor  Motor_gatherer   = null;
    public DcMotor  Motor_gatherer_lift   = null;
    public DcMotor  Motor_lift   = null;
    public DcMotor  Motor_wobble   = null;
    public Servo Servo_grabber   = null;
    public DigitalChannel Limit_wobble_up = null;
    public DigitalChannel Limit_wobble_down = null;
    public DigitalChannel Limit_lift_up = null;
    public DigitalChannel Limit_lift_down = null;
    public NormalizedColorSensor colorSensor;
    //public DistanceSensor sensorDistance = null;

    public static final double servo_middle   =  .5 ;
    public static final double servo_grabber_closed =  0 ;
    public static final double servo_grabber_open =  1 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware_11283(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and switches
        Motor_leftFront  = hwMap.get(DcMotor.class, "Front_left_wheel");
        Motor_leftRear = hwMap.get(DcMotor.class, "Rear_left_wheel");
        Motor_rightFront = hwMap.get(DcMotor.class, "Front_right_wheel");
        Motor_rightRear = hwMap.get(DcMotor.class, "Rear_right_wheel");
        Motor_gatherer  = hwMap.get(DcMotor.class, "Motor_gatherer");
        Motor_gatherer_lift = hwMap.get(DcMotor.class, "Motor_gatherer_lift");
        Motor_lift = hwMap.get(DcMotor.class, "Motor_lift");
        Motor_wobble  = hwMap.get(DcMotor.class, "Motor_wobble");

        Servo_grabber  = hwMap.get(Servo.class, "Servo_grabber");
        Limit_wobble_up  = hwMap.get(DigitalChannel.class, "Limit_wobble_up");
        Limit_wobble_down  = hwMap.get(DigitalChannel.class, "Limit_wobble_down");


       Limit_lift_up = hwMap.get(DigitalChannel.class, "Limit_lift_up");
       Limit_lift_down = hwMap.get(DigitalChannel.class, "Limit_lift_bottom");

        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensorColor");
        //sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");

        Motor_leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Motor_leftRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Motor_rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Motor_rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Motor_gatherer.setDirection(DcMotor.Direction.FORWARD);
        Motor_gatherer_lift.setDirection(DcMotor.Direction.REVERSE);
        Motor_lift.setDirection(DcMotor.Direction.FORWARD);
        Motor_wobble.setDirection(DcMotor.Direction.FORWARD);
        //set specific motors to halt immediately when the power is set to zero
        Motor_wobble.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Motor_lift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Motor_leftFront.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Motor_leftRear.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Motor_rightFront.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        Motor_rightRear.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        // Set all motors to zero power
        Motor_leftFront.setPower(0);
        Motor_leftRear.setPower(0);
        Motor_rightFront.setPower(0);
        Motor_rightRear.setPower(0);
        Motor_gatherer.setPower(0);
        Motor_gatherer_lift.setPower(0);
        Motor_lift.setPower(0);
        Motor_wobble.setPower(0);

        // Set motors to run with or without encoders.
        Motor_leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_gatherer_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_gatherer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Motor_example.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //set initial position of servos
       //
        // Servo_grabber.setPosition(servo_middle);



    }


}


