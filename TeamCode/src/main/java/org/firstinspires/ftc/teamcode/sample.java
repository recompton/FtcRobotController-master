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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DuckBlue", group="DanniJ")
//@Disabled
public class sample extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor LeftBack = null;
    DcMotor LeftFront = null;
    DcMotor RightBack = null;
    DcMotor RightFront = null;
    DcMotor Duck = null;

    // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_CORE = 288;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_REVS = (COUNTS_PER_MOTOR_CORE * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double SPIN_SPEED = 0.8;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        Duck = hardwareMap.dcMotor.get("Duck");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                LeftBack.getCurrentPosition(),
                LeftFront.getCurrentPosition(),
                RightBack.getCurrentPosition(),
                RightFront.getCurrentPosition());
        Duck.getCurrentPosition();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //THIS IS THE ONLY THING WE CHANGE EVERYTHING ELSE IS GOOD DON'T MESS IT UP

        // Step through each leg of the path,
        //sideways
        encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 5.0);
        //backwards
        encoderDrive(DRIVE_SPEED, 12, 12, -12, -12, 4.0);
        //spin Duck
        SpinDuck(SPIN_SPEED, -30, 4.0);
        //sideways to box
        encoderDrive(DRIVE_SPEED, 13, -13,13 , -13 , 5.0 );
        //backwards
        encoderDrive(DRIVE_SPEED, 4, 4, -4, -4, 4.0f);
       /*robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
       robot.rightClaw.setPosition(0.0);
       sleep(1000);     // pause for servos to move*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftBackInches, double leftFrontInches,
                             double rightBackInches, double rightFrontInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = RightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = RightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            LeftBack.setTargetPosition(newLeftBackTarget);
            LeftFront.setTargetPosition(newLeftFrontTarget);
            RightBack.setTargetPosition(newRightBackTarget);
            RightFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBack.isBusy() && LeftFront.isBusy() && RightBack.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newLeftFrontTarget, newRightBackTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        LeftBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        RightBack.getCurrentPosition(),
                        RightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            RightBack.setPower(0);
            RightFront.setPower(0);
            LeftBack.setPower(0);
            LeftFront.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void SpinDuck(double speed, double duckSpins,
                         double timeoutS) {
        int newDuckTarget;
        if (opModeIsActive()) {
            newDuckTarget = Duck.getCurrentPosition() + (int) (duckSpins * COUNTS_PER_REVS);
            Duck.setTargetPosition(newDuckTarget);

            Duck.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Duck.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Duck.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newDuckTarget);
                Duck.getCurrentPosition();
                telemetry.update();
            }
        }
    }
}

