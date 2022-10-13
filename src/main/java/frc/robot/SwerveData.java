// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

/** Add your docs here. */
public class SwerveData {
    public int driveCANID;
    public int steerCANID;
    public int canCoderCANID;
    public InvertType driveInvert;
    public InvertType steerInvert;
    public double angleOffset_Deg;
    public String name;

    /**
     * 
     * @param _driveCANID
     * @param _driveInvertType
     * @param _steerCANID
     * @param _steerInvertType
     * @param _canCoderCANID
     * @param _angleOffset_Deg
     */
    public SwerveData(String _name, int _driveCANID, InvertType _driveInvertType, int _steerCANID, InvertType _steerInvertType, int _canCoderCANID, double _angleOffset_Deg){
        driveCANID = _driveCANID;
        driveInvert = _driveInvertType;
        steerCANID = _steerCANID;
        steerInvert = _steerInvertType;
        canCoderCANID = _canCoderCANID;
        angleOffset_Deg = _angleOffset_Deg;
        name = _name;
    }
}
