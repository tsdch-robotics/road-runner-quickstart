/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hardware1 {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx armVert = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.5);

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double ARM_SPEED_UP = 0.7;
    static final double ARM_SPEED_DOWN = -0.35;
    static final double ARM_SPEED_HOLD = 0.08;;

    // Define a constructor that allows the OpMode to pass a reference to itself.

        /**
         * Initialize all the robot's hardware.
         * This method must be called ONCE when the OpMode is initialized.
         *
         * All of the hardware devices are accessed via the hardware map, and initialized.
         */
        public void init () {
            // Define and Initialize Motors (note: need to use reference to actual OpMode).
            frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");
            armVert = myOpMode.hardwareMap.get(DcMotorEx.class, "armVert");

            // set motor directions
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backRight.setDirection(DcMotorEx.Direction.REVERSE);

            //
            leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
            rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");

            myOpMode.telemetry.addData(">", "Hardware Initialized");
            myOpMode.telemetry.update();
        }

        /**
         * Calculates the left/right motor powers required to achieve the requested
         * robot motions: Drive (Axial motion) and Turn (Yaw motion).
         * Then sends these power levels to the motors.
         *
         * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
         */
        public void driveRobot ( double Drive, double Turn){
            // Combine drive and turn for blended motion.
            double left = Drive + Turn;
            double right = Drive - Turn;

            // Scale the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Use existing function to drive both wheels.
            setDrivePower(left, right);
        }

        /**
         * Pass the requested wheel motor powers to the appropriate hardware drive motors.
         *
         * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         */
        public void setDrivePower ( double leftWheel, double rightWheel){
            // Output the values to the motor drives.
            frontLeft.setPower(leftWheel);
            frontRight.setPower(rightWheel);
            backLeft.setPower(leftWheel);
            backRight.setPower(rightWheel);
        }

        /**
         * Pass the requested arm power to the appropriate hardware drive motor
         *
         * @param power driving power (-1.0 to 1.0)
         */
        public void setArmPower ( double power){
            armVert.setPower(power);
        }

    public static void sleep(long mls) throws InterruptedException {

    }

    public void encoderDrive(double speed, double speed2, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, int targetPos1, int targetPos2) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        /*frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armVert.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (targetPos1 * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (targetPos2 * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (targetPos1 * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (targetPos2 * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed2);
        backLeft.setPower(speed);
        backRight.setPower(speed2);
        }
    }