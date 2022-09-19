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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive Blue Full", group="Pushbot")

public class Auto_Drive_Blue_Full extends Auto_Drive_Common {

    /* Declare OpMode members. */




    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
       // robot.Motor_leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.Motor_leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.Motor_rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.Motor_rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        wobble_arm_in();
        grabber_closed();
        DriveForwardUntilBlueLine(5,.5);
        moveForward(1,.5);  //this moves the color sensor off the blue line
        DriveForwardUntilBlueLine(3,.5);
        moveForward(1.5,.5);
        wobble_arm_out();
        grabber_open();
        wobble_arm_in();  //this gets the arm out of the way so it doesn't drag the wobble
        DriveForwardUntilBlueLine(1.7,.5);
        wobble_arm_out();  //this gets the wobble arm out of the way so the lift doesn't catch it on the way up
        moveForward(1,.5);
        DriveForwardUntilBlueLine(3,.5);
        moveRight(1,.5);
        lift_up();
        lift_down();
        moveLeft(1.5,.5);  //this moves to the side so it doesn't hit the wobble on the way back to the white line
        DriveBackwardUntilWhiteLine(5,.5);




}





}
