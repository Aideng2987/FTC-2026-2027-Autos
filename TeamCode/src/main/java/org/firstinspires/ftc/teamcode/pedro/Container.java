package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


public class Container {
    // Drivetrain
    public DcMotor frontLeft, backLeft, frontRight, backRight;

    // Mechanisms
    public DcMotor shooterMotor;
    public DcMotor intakeMotor;
    public CRServo leftIntakeServo;
    public CRServo rightIntakeServo;
    public CRServo mainIntakeServo;
    private final PoseFactory p = PoseFactory.degrees();
    public final Pose start          = p.of(58.75, 13.75, 90);
    public final Pose shoot          = p.of(58.75, 12.75, 270);
    public final Pose intake         = p.of(12.75, 10.75, 180);
    public final Pose turn           = p.of(17.75, 21.75, 180);
    public final Pose shootAgain     = p.of(58.75, 13.75, 270);
    public final Pose bumpIntoFlower = p.of(14.75, 43.75, -34.2869);
    public final Pose parking        = p.of(10.75, 100.75, -85.9858);

    public Path shootPath()          { return Paths.line(start, shoot).linear(start, shoot); }
    public Path intakePath()         { return Paths.line(shoot, intake).constant(intake); }
    public Path turnPath()           { return Paths.line(intake, turn).constant(turn); }
    public Path shootAgainPath()     { return Paths.line(turn, shootAgain).constant(shootAgain); }
    public Path bumpIntoFlowerPath() { return Paths.line(shootAgain, bumpIntoFlower).reverseTangent(); }
    public Path parkingPath()        { return Paths.line(bumpIntoFlower, parking).reverseTangent(); }


    // Call in  TeleOp and Auto
    public void initMechanisms(HardwareMap hw) {
        shooterMotor = hw.get(DcMotor.class, "shooterMotor");
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeServo = hw.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hw.get(CRServo.class, "rightIntakeServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        mainIntakeServo = hw.get(CRServo.class, "mainIntakeServo");
        mainIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Calling only  in TeleOp
    public void initDrivetrain(HardwareMap hw) {
        frontLeft = hw.get(DcMotor.class, "topLeftWheel");
        backLeft = hw.get(DcMotor.class, "bottomLeftWheel");
        frontRight = hw.get(DcMotor.class, "topRightWheel");
        backRight = hw.get(DcMotor.class, "bottomRightWheel");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobot(double y, double x, double rx) {
        // the standard dive train stuff
        double fl = (y + x) + rx;
        double bl = (y - x) + rx;
        double fr = (y - x) - rx;
        double br = (y + x) - rx;

        double max = Math.max(Math.abs(fl), Math.abs(fr));
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void shoot(double shotPower) {
        shooterMotor.setPower(shotPower);
    }
    // probably need to tune this so that we don't overshoot with the power and maybe we can get a warm going

    public void intake(double power){
        intakeMotor.setPower(power);
        leftIntakeServo.setPower(-(power));
        rightIntakeServo.setPower((power));
        mainIntakeServo.setPower(power);
    }

    public Command shootForMs(long ms) {
        return sequential(
                instant(() -> shoot(1.0)),
                waitMs(ms),
                instant(() -> shoot(0.0))
        );
    }

    public Command intakeForMs(long ms, double power) {
        return sequential(
                instant(() -> intake(power)),
                waitMs(ms),
                instant(() -> intake(0.0))
        );
    }
    public Command autoRoutine(Follower follower) {
        return sequential(
                follow(follower, shootPath()),
                waitMs(2000.0),
                shootForMs(1500),
                parallel( // this is stuff that we do while doing smt else. Kinda multi taksing the robot
                        sequential(
                                follow(follower, intakePath()),
                                follow(follower, turnPath())
                        ),
                        intakeForMs(3000, 1.0)
                ),
                follow(follower, shootAgainPath()),
                waitMs(2000.0),
                shootForMs(1500),
                follow(follower, bumpIntoFlowerPath()),
                follow(follower, parkingPath())
        );
    }

}