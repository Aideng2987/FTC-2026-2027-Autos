package org.firstinspires.ftc.teamcode.pedro.procedures;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Container;


import static com.pedropathing.ivy.Scheduler.schedule;
@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {
    private Follower follower;
    private final Container robot = new Container();
    private Command auto = null;


    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(robot.start);
        follower.update();
        robot.initMechanisms(hardwareMap);
        robot.initDrivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            boolean autoRunning = (auto != null && auto.isScheduled());
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            double intake1 = gamepad1.left_trigger;
            double fire = gamepad1.right_trigger;
            // press a to start the auto
            if (gamepad1.a && !autoRunning) {
                follower.setPose(robot.start);
                auto = robot.autoRoutine(follower);
                schedule(auto);
                autoRunning = true;
            }

            //  Press B or move sticks to immediately abort auto and return to manual driving

            boolean sticksMoved = Math.abs(y) > 0.1 || Math.abs(x) > 0.1 || Math.abs(rx) > 0.1;
            if (autoRunning && (gamepad1.b || sticksMoved)) {
                Scheduler.reset();
                robot.intake(0.0);
                robot.shoot(0.0);
                auto = null;
                autoRunning = false;
            }

            // Manual driving & mechanisms only when auto is not active
            if (!autoRunning) {
                robot.driveRobot(y, x, rx);
                if (intake1 > 0.1) {
                    robot.intake(intake1);
                } else if (gamepad1.dpad_down) {
                    robot.intake(-0.8);
                } else {
                    robot.intake(0.0);
                }

                // Shooter right trigger = shoot
                if(fire > 0.1){
                robot.shoot(fire);}
                else{
                    robot.shoot(0.0);
                }
            }
            telemetry.addData("Status", "Right Trigger: %.2f | Auto Running: %b", gamepad1.right_trigger, autoRunning, follower);
            telemetry.update();
        }
    }
}