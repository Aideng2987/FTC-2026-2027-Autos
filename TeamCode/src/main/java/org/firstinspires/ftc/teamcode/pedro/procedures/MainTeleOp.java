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

        waitForStart();

        while (opModeIsActive()) {

            boolean autoRunning = (auto != null && auto.isScheduled());

            if(gamepad1.a && (!autoRunning)){
                follower.setPose(robot.start);
                auto = robot.autoRoutine(follower);
                schedule(auto);
            }
            else{
                robot.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            // intake: Left Trigger = in d-pad Down = out
            if (gamepad1.left_trigger > 0.1) {
                robot.intake(gamepad1.left_trigger);
            } else if (gamepad1.dpad_down) {
                robot.intake(-0.8);
            } else {
                robot.intake(0.0);
            }

            // Shooter: right Trigger = shoot
            robot.shoot(gamepad1.right_trigger);}

            telemetry.addData("Inputs", "L-Trig:  R-Trig: | D-Down: ",
                    gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.dpad_down);
            telemetry.update();
        }
    }
}