package frc.com.simulator;

import java.util.ArrayList;
import java.util.List;

public final class Simulator {

    private static volatile Simulator instance;

    public List<Simulatable> simulatables;
    
    private Simulator() {
        simulatables = new ArrayList<>();
    }

    public static Simulator getInstance() {
        Simulator result = instance;
        if (result != null) {
            return result;
        }
        synchronized(Simulator.class) {
            if (instance == null) {
                instance = new Simulator();
            }
            return instance;
        }
    }

    public void addToSimulator(Simulatable simulatable){
        simulatables.add(simulatable);
    }
    
    public void run(){
        for(var simulatable : simulatables){
            simulatable.update();
        }
    }
}
