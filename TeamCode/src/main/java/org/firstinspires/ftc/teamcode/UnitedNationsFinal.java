package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="United Nation Final", group = "Linear Op Mode")

public class UnitedNationsFinal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor motorArm = null;
    private DcMotor motorLift1 = null;
    private DcMotor motorLift2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        motorArm = hardwareMap.get(DcMotor.class,"motorArm");
        motorLift1 = hardwareMap.get(DcMotor.class,"motorLift1");
        motorLift2 = hardwareMap.get(DcMotor.class,"motorLift2");
        double armPower = 0;
        double armReversePower = 0;
        int armUpPosition = 1000;
        int armDownPosition = 0;


        //Need to reverse one of the other wheels since new config
        //Need to reverse one of the the motors that lift the arm
        //Potentially need to reverse the arm.

        //Wheels
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        motorLift1.setDirection(DcMotor.Direction.REVERSE);
        motorLift2.setDirection(DcMotor.Direction.FORWARD);

        //Lift movement:

        motorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift1.setTargetPosition(armDownPosition);
        motorLift2.setTargetPosition(armDownPosition);
        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;


            //Extending arm

            armPower = gamepad1.right_trigger * 0.5;
            armReversePower = gamepad1.left_trigger * 0.5 ;
            motorArm.setPower(armPower);
            motorArm.setPower(-armReversePower);

            //Lifting up the arm

            if(gamepad2.a) {
                motorLift1.setTargetPosition(armUpPosition);
                motorLift2.setTargetPosition(armUpPosition);
                motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift1.setPower(0.5);
                motorLift2.setPower(0.5);
            }

            //Lowering the arm

            motorLift1.setTargetPosition(armDownPosition);
            motorLift2.setTargetPosition(armDownPosition);
            motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift1.setPower(0.5);
            motorLift2.setPower(0.5);



            // uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 0.5;
            double yaw     =  gamepad1.right_stick_x * 0.5;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);


            //Position of the Encoders

            double position1 = motorLift1.getCurrentPosition();
            double position2 = motorLift2.getCurrentPosition();

            //Target position of encoders

            double desiredPosition1 =  motorLift1.getTargetPosition();
            double desiredPosition2 =  motorLift1.getTargetPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Lift Power", armReversePower);
            telemetry.addData("Encoder Position", position1);
            telemetry.addData("Encoder Position", position2);
            telemetry.addData("Desired Position", desiredPosition1);
            telemetry.addData("Desired Position", desiredPosition2);

            telemetry.update();

        }

    }}

