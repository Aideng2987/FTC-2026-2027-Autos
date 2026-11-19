package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumDriveLocalizer;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("topLeftWheel");
                c.backLeftName.set("bottomLeftWheel");
                c.frontRightName.set("topRightWheel");
                c.backRightName.set("bottomRightWheel");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            }
    );

    // Blank placeholder until you paste the generated config from ForesightTuner
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {});

    public static Follower create(HardwareMap h) {
        return new Follower(
                new MecanumDriveLocalizer(h),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}