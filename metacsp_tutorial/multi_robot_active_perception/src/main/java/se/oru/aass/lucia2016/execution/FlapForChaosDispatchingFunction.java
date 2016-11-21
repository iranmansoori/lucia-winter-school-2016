package se.oru.aass.lucia2016.execution;

import java.util.Calendar;
import java.util.Vector;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.multi.activity.SymbolicVariableActivity;

public class FlapForChaosDispatchingFunction extends DispatchingFunction {

	public static final Vector<SymbolicVariableActivity> executingActs = new Vector<SymbolicVariableActivity>();
	
	public FlapForChaosDispatchingFunction(String component) {
		super(component);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean skip(SymbolicVariableActivity act) {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	public void dispatch(SymbolicVariableActivity act) {
		// TODO Auto-generated method stub
		System.out.println(">>> " + this.getComponent() + " starts executing " + act);
		executingActs.add(act);
		if (act.getSymbols()[0].equals("Sense")) {
			final long sleepTime = act.getTemporalVariable().getDuration().min;
			final FlapForChaosDispatchingFunction thisDF = this;
			final SymbolicVariableActivity theAct = act;
			new Thread() {
				public void run() {
					long start = Calendar.getInstance().getTimeInMillis();
					while (Calendar.getInstance().getTimeInMillis() - start < sleepTime) {
						try { Thread.sleep(10); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					thisDF.finish(theAct);
					executingActs.remove(theAct);
				}
			}.start();
		}
	}
	
}
