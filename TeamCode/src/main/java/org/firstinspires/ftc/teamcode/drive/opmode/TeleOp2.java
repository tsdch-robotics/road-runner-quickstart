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

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx armVert = null;

    private Servo leftHand = null;
    private Servo rightHand = null;

    final int highGoal = 4000;
    final int medGoal = 2800;
    final int lowGoal = 1300;
    double armPower = 0;
    private int most = 0;

    double minposL = 0.4, maxposL = 0.73, minposR = 0.72, maxposR = 0.95;

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
        //armVert.setDirection(DcMotorEx.Direction.REVERSE);

        //armVert.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


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

            //leftPower  = gamepad1.left_stick_y ;
            //rightPower = gamepad1.right_stick_y ;
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
            //set armvertpower
            if (gamepad1.right_trigger > 0) {
                armPower = -0.55   ;
            }
            else if (gamepad1.left_trigger > 0) {
                armPower = 0.95;
            }
            else {
                armPower = 0.11;
            }


            //open the gripper on X button if not already at most open position.
            if (gamepad1.left_bumper && gripposR < maxposR) gripposR = gripposR + .01;

            // close the gripper on Y button if not already at the closed position.
            if (gamepad1.right_bumper && gripposR > minposR) gripposR = gripposR - .01;


            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            armVert.setPower(armPower);

            leftHand.setPosition(gripposR);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
