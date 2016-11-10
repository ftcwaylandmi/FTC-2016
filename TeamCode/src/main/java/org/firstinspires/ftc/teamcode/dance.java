/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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
 * Remove or comment out the @Disabled line to add this +opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Dance", group="Pushbot")
//@Disabled
public class dance extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    float           forwardPower = -1;
    float           reversePower = 1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        AuxMotor1Forward();
        Sleeper(100);
        AuxMotor1Reverse();
        Sleeper(100);
        SpinLeft(3000);
        AuxMotor1Stop();
        SpinRight(3000);
        DriveForward(1000);
        AuxMotor1Forward();
        SpinLeft(2000);
        AuxMotor1Stop();
        DriveStop();


    }

    public void AuxMotor1Stop() {
        robot.armMotor.setPower(0);
    }

    public void AuxMotor1Forward() {
        robot.armMotor.setPower(forwardPower);
    }

    public void AuxMotor1Reverse() {
        robot.armMotor.setPower(reversePower);
    }



    public void SpinLeft(int drivetime) {
        robot.leftMotor.setPower(reversePower);
        robot.rightMotor.setPower(forwardPower);
        Sleeper(drivetime);
    }

    public void SpinRight(int drivetime) {
        robot.leftMotor.setPower(forwardPower);
        robot.rightMotor.setPower(reversePower);
        Sleeper(drivetime);
    }

    public void SweepTurnLeft90(){
        SweepTurnLeft(1500);
    }

    public void SweepTurnRight90(){
        SweepTurnRight(1500);
    }

    public void SweepTurnRight(int drivetime) {
        //FIXME reversed due to motors being mislabeled.
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(forwardPower);
    }

    public void SweepTurnLeft(int drivetime) {
        //FIXME reversed due to motors being mislabeled.
        robot.leftMotor.setPower(forwardPower);
        robot.rightMotor.setPower(0);
        Sleeper(drivetime);
    }


    public void DriveStop() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }


    public void Sleeper(int sleeptime) {
        try {
            Thread.sleep(sleeptime);

        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public void DriveReverse(int drivetime) {
        robot.leftMotor.setPower(reversePower);
        robot.rightMotor.setPower(reversePower);
        Sleeper(drivetime);
    }


    public void DriveForward(int drivetime) {
        robot.leftMotor.setPower(forwardPower);
        robot.rightMotor.setPower(fowardPower);
        Sleeper(drivetime);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
