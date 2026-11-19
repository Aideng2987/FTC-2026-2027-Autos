package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumDriveLocalizer;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new MecanumDriveLocalizer(hardwareMap),
                () -> new Foresight(Constants.foresightConfig)
        );
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new MecanumDriveLocalizer(hardwareMap),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }
}