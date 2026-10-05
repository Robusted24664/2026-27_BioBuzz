package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="HoodTest", group="Testing")
public class HoodTest extends LinearOpMode{

    // Declare OpMode members.
    private Servo hood = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        hood = hardwareMap.get(Servo.class, "hood");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double hoodPos = 0.5;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
        if (gamepad1.xWasPressed()) {
            hoodPos = 0.95;

        }
        
        if (gamepad1.yWasPressed()) {
            hoodPos = 0.18;

        }
        
        hoodPos += gamepad1.left_stick_y / 5000;

        hoodPos = Range.clip(hoodPos, 0.18, 0.99);

        hood.setPosition(hoodPos);
            
            
            
            // Send telemetry data to the driver station
            telemetry.addData("hood Position", hoodPos);


            telemetry.update();
        }
    }
}
