// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import frc.robot.RobotContainer;
import frc.robot.Constants.OI;

/** Add your docs here. */
public class Util {
    private static double m_robotDesiredAngle = 0 ;
    public static double deadband(double val, double limit){
        double rtn = 0;
        
        rtn = Math.abs(val);
        rtn = (rtn >= limit) ? (rtn - limit) * (1 + limit) : 0;

     
        return rtn * Math.signum(val);
    }
    public static double limit(double _value, double _min, double _max) {
        double rtn = _value;
        if(_value < _min) {
            rtn = _min;
        }else if(_value > _max){
            rtn = _max;
        }
        return rtn;
    }
    public static boolean hyp(double _a, double _b){
        if(Math.sqrt(_a * _a + _b * _b) > OI.kCMax){
            return true;
        }else {
            return false;
        }
        

    }
    public static double getAngle( double _a, double _b){
        double a = Math.abs(_a);
        
        double ang = 0;
        double c = Math.sqrt(_a * _a + _b * _b);
        
        double alpha = Math.toDegrees(Math.asin(a/c));
        if(_a >= 0 && _b >= 0){ // Quad 1
            ang = alpha;
        }else if(_a > 0 && _b < 0){ //Quad 2
            ang = 180 - alpha;
        }else if(_a < 0 && _b < 0){ //Quad 3
            ang = 180 + alpha;
        }else if(_a < 0 && _b > 0){ //Quad 4
            ang = 360 - alpha;
        }
        if(ang> 180){
            ang -= 360;
        }
        if(hyp(_a, _b) && RobotContainer.stick.getRawButton(10)) {
            m_robotDesiredAngle = -ang;
        }
        return m_robotDesiredAngle;
    }
}