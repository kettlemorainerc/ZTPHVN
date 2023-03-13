package org.usfirst.frc.team2077.common.subsystem;

import java.util.HashMap;
import java.util.Map;

public class InputMap { //This is probably a bad I idea but I like it in the moment

    private static Map<String, DoubleProvider> map = new HashMap<String, DoubleProvider>();

    public static void bindAxis(String key, DoubleProvider lambda){map.put(key, lambda);}
//    public void bindAxis(DoubleProvider lambda) {bindAxis("", lambda);}

    public static double getInput(String key){return map.get(key).get();}
//    public double getInput(){return getInput("");}

    public interface DoubleProvider {double get();}

}
