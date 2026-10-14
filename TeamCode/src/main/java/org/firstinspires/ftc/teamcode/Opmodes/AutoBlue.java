package org.firstinspires.ftc.teamcode.Opmodes;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.subsys.MecanumDrive;


@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")

public class AutoBlue extends LinearOpMode {

    /* Declare OpMode members. */


    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.


    private MecanumDrive drive;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.


        // Send telemetry message to indicate successful Encoder reset
        drive = new MecanumDrive(hardwareMap);

        // Wait for the game to start (driver presses START)
        waitForStart();

        if (opModeIsActive()) {

            drive.driveDistanceCM(50.0, 0.6);


            drive.strafeDistanceCM(-20.0, 0.4);

            drive.turnDegrees(90.0, 1.0);
        }




        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */


}
