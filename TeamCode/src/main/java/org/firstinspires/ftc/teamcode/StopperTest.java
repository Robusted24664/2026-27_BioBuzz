package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="StopperTest", group="Testing")
public class StopperTest extends LinearOpMode{

    // Declare OpMode members.
    private Servo leftStopper = null;
    private Servo rightStopper = null;
    

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        leftStopper = hardwareMap.get(Servo.class, "leftStopper");
        rightStopper = hardwareMap.get(Servo.class, "rightStopper");
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftStopperPos = 0.5;
        double rightStopperPos = 0.5;
        

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
        if (gamepad1.xWasPressed()) {
            leftStopperPos = 0.25;
            rightStopperPos = 0.29;
            
        }
        
        if (gamepad1.yWasPressed()) {
            leftStopperPos = 0.61;
            rightStopperPos = 0.64;
            
        }
        
        leftStopperPos += gamepad1.left_stick_y / 5000;
        rightStopperPos += gamepad1.right_stick_y / 5000;
            
        leftStopperPos = Range.clip(leftStopperPos, 0.0, 1.0);
        rightStopperPos = Range.clip(rightStopperPos, 0.0 , 1.0);
        
        leftStopper.setPosition(leftStopperPos);
        rightStopper.setPosition(rightStopperPos);
        


            
            
            
            // Send telemetry data to the driver station
            telemetry.addData("leftStopper Position", leftStopperPos);
            telemetry.addData("rightStopper Position", rightStopperPos);


            telemetry.update();
        }
    }
}
