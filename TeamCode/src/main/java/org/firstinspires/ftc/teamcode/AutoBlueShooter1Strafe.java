package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;

@Autonomous(name = "Auto Blue Shooter 1 Strafe", group = "auto", preselectTeleOp="Decode Teleop")
public class AutoBlueShooter1Strafe extends LinearOpMode {
    public RoRobot robot = new RoRobot();
 
    public double start_time = 0;

 
    public void waitForFlywheelVelocity(double targetVelocity) {
        while (opModeIsActive()) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.addData("target flywheel velocity:", targetVelocity);
            telemetry.addData("Time:", getRuntime() - start_time);

            telemetry.update();
            if (flywheel_velocity > targetVelocity) {
                break;
                
            }
        }
    }
    
    public void fireSequence(double targetVelocity, double targetPower, double waitingPower) {
        robot.flywheelSetPower(targetPower);
        waitForFlywheelVelocity(targetVelocity);
        robot.flywheelBoost();
        robot.autoFiringStateFire();
        sleep (500);
        robot.flywheelSetPower(waitingPower);
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        DriveParameters mediumFastForward = robot.getDriveParametersForMediumFastForward();

        
        robot.setBlue();

        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        start_time = getRuntime();
        
        double velocityMultiplier = 6000.0*0.85*28.0/60.0;
        double targetVelocity = 1050;
        double targetPower = targetVelocity/velocityMultiplier;
        
        double waitingPower = targetPower*0.9;

        
        robot.flywheelSetPower(targetPower);
        robot.firingStateMaintain();
        robot.hoodPos(0.8346);
        
        robot.setBrakes();
        
        if (robot.poseEst.position.y < 0.0) {
            if (robot.poseEst.heading > -0.25 * Math.PI && robot.poseEst.heading < 0.25 * Math.PI) {
                RoPose TargetPose = robot.poseEst.thenForward(-10);
                robot.strafeToPoseAndStop(TargetPose, mediumForward);
                robot.strafeToPoseAndStopD(-15,-40,180-90, mediumForward);
            }
            robot.strafeToPoseAndStopD(-20,20,180-90, fastForward);
        }
        
        

        robot.strafeToPoseAndStopD(-31.169,34.541,180-42.6, slowForward);
        
        
        fireSequence(targetVelocity, targetPower, waitingPower);
        
        //collect first line

        robot.firingStateIntake();
        robot.hoodPos(0.8346);
        
        

        robot.strafeToPoseAndStopD(-30.75,13.40,180-0, fastForward); // allign first line
        robot.strafeToPoseAndStopD(-54.5,13.40,180-0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(-35.0,13.40,180-0.0, fastForward); // move back
        
        robot.strafeToPoseAndStopD(-32,27,180-42.6, mediumForward); // move close to fire pos
        robot.strafeToPoseAndStopD(-29.169,32.541,180-42.6, slowForward); // move to fire pos
        
        fireSequence(targetVelocity, targetPower, waitingPower);
        
        //collect second line

        robot.firingStateIntake();
        robot.hoodPos(0.8346);

        robot.strafeToPoseAndStopD(-30.43,-9.64,180-0, fastForward); // allign second line
        robot.strafeToPoseAndStopD(-61.25,-9.64,180-0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(-35.0,-9.64,180-0.0, fastForward); // move back
        
        robot.strafeToPoseAndStopD(-29.169,32.541,180-42.6, mediumFastForward); // move to fire pos
        
        fireSequence(targetVelocity, targetPower, waitingPower);

        
        //collect third line

        robot.firingStateIntake();
        robot.hoodPos(0.8346);

        robot.strafeToPoseAndStopD(-29.7,-34.8,180-0, fastForward); // allign third line
        robot.strafeToPoseAndStopD(-61.25,-34.8,180-0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(-35.0,-34.8,180-0.0, fastForward); // move back
        
        
        robot.strafeToPoseAndStopD(-29.169,32.541,180-42.6, mediumFastForward); // move to fire pos
        
        fireSequence(targetVelocity, targetPower, waitingPower);

        robot.strafeToPoseAndStopD(-48.0,-1.0,180-180, mediumFastForward);

    }
}
