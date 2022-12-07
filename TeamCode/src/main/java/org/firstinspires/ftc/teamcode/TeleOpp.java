package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "TeleOp", group = "DanniJ")
public class TeleOpp extends OpMode {
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    /*
     * Constructor
     */

    // Code to run when the op mode is first enabled goes here
    @Override
    public void init() {


        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */

        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        /*
         * Gamepad 1
         *
         * Gamepad 1 controls the motors via the left and right sticks
         */

        //throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        //1 is full down
        //direction: left_stick_x ranges from -1 to 1, where -1 is full left
        //and 1 is full right
        //these are for the drive motors
        float throttleL = -gamepad1.left_stick_y;
        float sidewaysL = gamepad1.left_stick_x;
        float throttleR = -gamepad1.right_stick_y;
        float sidewaysR = -gamepad1.right_stick_x;
        //scale the joystick value to make it easier to control
        //the robot more precisely at slower speeds.
        throttleR = (float)scaleInput(throttleR);
        throttleL = (float)scaleInput(throttleL);
        sidewaysR = (float)scaleInput(sidewaysR);
        sidewaysL = (float)scaleInput(sidewaysL);

        //write the values to the motors
        motorRightFront.setPower(throttleR);
        motorLeftFront.setPower(throttleL);
        motorRightBack.setPower(throttleR);
        motorLeftBack.setPower(throttleL);
        //sideways
        motorRightFront.setPower(sidewaysL);
        motorLeftBack.setPower(sidewaysL);
        motorLeftFront.setPower(sidewaysR);
        motorRightBack.setPower(sidewaysR);

        //drive motors ends here

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Running!");
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 3.0};

        //get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 22.0);

        //index should be positive.
        if (index < 0) {
            index = -index;
        }

        //index cannot exceed size of array minus 1.
        if (index > 22) {
            index = 22;
        }

        //get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        }
        else {
            dScale = scaleArray[index];
        }

        //return scaled value.
        return dScale;
    }

}
