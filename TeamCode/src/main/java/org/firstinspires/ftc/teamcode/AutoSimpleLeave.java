package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RoRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;


// normal starting point at (33.4, 31.5, 90)

@Autonomous(name="SimpleLeave", group="auto", preselectTeleOp="Decode Teleop")
public class AutoSimpleLeave extends LinearOpMode {
    public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);
        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        
        waitForStart();
        
        robot.setPoseD(0, 0, 0);
        
        robot.strafeToPoseAndStopD(-10,0,0, slowForward);
        
        sleep (1000);
        
    }
}
