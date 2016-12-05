package se.oru.aass.lucia2016.test;

import org.metacsp.framework.ConstraintNetwork;

public class OpenCNDebug {
	
	public static void main(String[] args) {
		ConstraintNetwork.draw(ConstraintNetwork.loadConstraintNetwork("testingLucia.cn"));
	}

}
