package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="UN: Autonomous For topside", group="Robot")
//@Disabled
public class UNAutonomousRed extends LinearOpMode {
    //For Red side starting on the top

    /* Declare OpMode members. */

    hardwareMap robot = new hardwareMap();
    private final ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.init(hardwareMap);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Drive forward for 2 seconds
        robot.leftFrontMotor.setPower(1);
        robot.leftBackMotor.setPower(1);
        robot.rightFrontMotor.setPower(1);
        robot.rightBackMotor.setPower(1);
        sleep(2000);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}

