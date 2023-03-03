package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.Hardware1;

@TeleOp(name="TeleOp2", group="Linear Opmode")
public class  TeleOp2 extends LinearOpMode {

    FtcDashboard dashboard;

    //lift state
    public enum LiftState {
        ENCODER,
        MANUAL
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.ENCODER;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx armVert = null;

    private Servo leftHand = null;
    private Servo rightHand = null;

    final int highGoal = 980;
    final int midGoal = 710;
    final int lowGoal = 440;
    final int pickup = 110;
    static final double upSpeed = 0.85;
    static final double downSpeed = -0.2;
    double armPower = 0;
    private int most = 0;

    double minposL = 0.4, maxposL = 0.73, minposR = 0.65, maxposR = 0.85;

    double gripposL = 0.7, gripposR = 0.65;
    double adjustedPos = maxposR;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized yuh");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        armVert = hardwareMap.get(DcMotorEx.class, "armVert");

        leftHand = hardwareMap.get(Servo.class, "left_hand");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        armVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //armVert.setTargetPosition(pickup);
        //armVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double drive = gamepad1.left_stick_y;
            double turn  =  0.7 * -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            double denom = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
            double frontLeftPower = (drive + turn + strafe) / denom;
            double frontRightPower = (drive - turn - strafe) / denom;
            double backLeftPower = (drive + turn - strafe) / denom;
            double backRightPower = (drive - turn + strafe) / denom;

/*
            if (gamepad1.a) {
                armVert.setTargetPosition(lowGoal);
                if (armVert.getCurrentPosition() > lowGoal) {
                    armVert.setPower(-0.45);
                }
                if (armVert.getCurrentPosition() < lowGoal) {
                    armVert.setPower(0.8);
                }
            }
            if (gamepad1.b) {
                armVert.setTargetPosition(medGoal);
                if (armVert.getCurrentPosition() > medGoal) {
                    armVert.setPower(-0.45);
                }
                if (armVert.getCurrentPosition() < medGoal) {
                    armVert.setPower(0.8);
                }
            }
            if (gamepad1.y) {
                armVert.setTargetPosition(highGoal);
                if (armVert.getCurrentPosition() > highGoal) {
                    armVert.setPower(-0.45);
                }
                if (armVert.getCurrentPosition() < highGoal) {
                    armVert.setPower(0.8);
                }
            }
            if (gamepad1.right_stick_button) {
                armVert.setTargetPosition(pickup);
                if (armVert.getCurrentPosition() > pickup) {
                    armVert.setPower(-0.45);
                }
                if (armVert.getCurrentPosition() < pickup) {
                    armVert.setPower(0.8);
                }
            }
*/

            switch (liftState) {
                case ENCODER:
                    armVert.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    if (gamepad1.a) {
                        armVert.setTargetPosition(lowGoal);
                        armVert.setPower(upSpeed);

                    }
                    if (gamepad1.b) {
                        armVert.setTargetPosition(midGoal);
                        if (armVert.getCurrentPosition() > midGoal) {
                            armVert.setTargetPosition(midGoal);
                            armVert.setPower(downSpeed);
                        }
                        if (armVert.getCurrentPosition() < midGoal) {
                            armVert.setTargetPosition(lowGoal);
                            armVert.setPower(upSpeed);
                        }
                    }
                    if (gamepad1.y) {
                        armVert.setTargetPosition(highGoal);
                        if (armVert.getCurrentPosition() > highGoal) {
                            armVert.setTargetPosition(highGoal);
                            armVert.setPower(downSpeed);
                        }
                        if (armVert.getCurrentPosition() < highGoal) {
                            armVert.setTargetPosition(highGoal);
                            armVert.setPower(upSpeed);
                        }
                    }
                    if (gamepad1.right_stick_button) {
                        armVert.setTargetPosition(pickup);
                        if (armVert.getCurrentPosition() > pickup) {
                            armVert.setTargetPosition(pickup);
                            armVert.setPower(downSpeed);
                        }
                        if (armVert.getCurrentPosition() < pickup) {
                            armVert.setTargetPosition(pickup);
                            armVert.setPower(upSpeed);
                        }
                    }

                    if (gamepad1.dpad_right) {
                        liftState = LiftState.MANUAL;
                    }
                    break;
                case MANUAL:
                    armVert.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        if (gamepad1.right_trigger > 0) {
                            armPower = -0.3;
                        }
                        else if (gamepad1.left_trigger > 0) {
                            armPower = 0.85;
                        }
                        else {
                            armPower = 0.03;
                        }

                        if (gamepad1.dpad_left) {
                            liftState = LiftState.ENCODER;
                        }

                    armVert.setPower(armPower);
                    }



            //set armvertpower


            //open the gripper on X button if not already at most open position.
            if (gamepad1.left_bumper && gripposR < maxposR) gripposR = gripposR + .01;

            // close the gripper on Y button if not already at the closed position.
            if (gamepad1.right_bumper && gripposR > minposR) gripposR = gripposR - .01;


            frontLeft.setPower(0615+26+156);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);


            leftHand.setPosition(gripposR);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
