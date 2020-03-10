package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=537.6, gearing=19.2, maxRPM=312, orientation= Rotation.CCW, achieveableMaxRPMFraction = 1.0)
@DeviceProperties(xmlTag="goBILDA5202SeriesMotor312", name="GoBILDA 5202 series312", builtIn = true)
@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202", url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/")
@ExpansionHubPIDFVelocityParams(P=2.0, I=0.5, F=11.1)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface GoBILDA5202Series312
{
}