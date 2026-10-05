package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Position Tester", group = "Test Group")

public class PositionTester extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        robot.setPoseD(33.4, 31.5, 90);
        robot.reportPositionData();

        waitForStart();
        
        while (opModeIsActive()) {
            robot.updatePose();
            robot.reportPositionData();
        }
    }
}
