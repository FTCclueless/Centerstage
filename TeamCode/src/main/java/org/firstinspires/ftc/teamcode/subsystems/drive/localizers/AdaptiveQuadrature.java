package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

public class AdaptiveQuadrature {
	private double[] vel, heading;
	public AdaptiveQuadrature(double[] vel, double[] heading) {
		this.vel = vel;
		this.heading = heading;
	}

	private double fcos(double t) {
		if (t == 0) {
			return vel[0]*Math.cos(heading[0]);
		}
		double v = 0;
		for (int i = 0; i < vel.length; i ++)
			v += vel[i] * Math.pow(t,i);
		double h = 0;
		for (int i = 0; i < heading.length; i ++)
			h += heading[i] * Math.pow(t,i);
		return v*Math.cos(h);
	}

	private double fsin(double t) {
		if (t == 0) {
			return vel[0]*Math.sin(heading[0]);
		}
		double v = 0;
		for (int i = 0; i < vel.length; i ++)
			v += vel[i] * Math.pow(t,i);
		double h = 0;
		for (int i = 0; i < heading.length; i ++)
			h += heading[i] * Math.pow(t,i);
		return v*Math.sin(h);
	}

	public double evaluateCos(double eps, double t1, double t2, int level) { //vel*cos(heading)
		double s1 = (t2-t1)*(fcos(t1)+4.0*fcos((t1+t2)/2.0)+fcos(t2))/6.0;
		double s2 = (t2-t1)*(fcos(t1)+4.0*fcos(0.75*t1+0.25*t2)+2.0*fcos(0.5*t1+0.5*t2)+4.0*fcos(0.25*t1+0.75*t2)+fcos(t2))/12.0;
		if (Math.abs(s2-s1) <= eps) {
			return s2;
		}
		if (level > 10) {
			return s2;
		}
		return evaluateCos(eps/2.0,t1,(t1+t2)/2.0,level+1) + evaluateCos(eps/2.0,(t1+t2)/2.0,t2,level+1);
	}

	public double evaluateSin(double eps, double t1, double t2, int level) { //vel*sin(heading)
		double s1 = (t2-t1)*(fsin(t1)+4.0*fsin((t1+t2)/2.0)+fsin(t2))/6.0;
		double s2 = (t2-t1)*(fsin(t1)+4.0*fsin(0.75*t1+0.25*t2)+2.0*fsin(0.5*t1+0.5*t2)+4.0*fsin(0.25*t1+0.75*t2)+fsin(t2))/12.0;
		if (Math.abs(s2-s1) <= eps) {
			return s2;
		}
		if (level > 10) {
			return s2;
		}
		return evaluateSin(eps/2.0,t1,(t1+t2)/2.0,level+1) + evaluateSin(eps/2.0,(t1+t2)/2.0,t2,level+1);
	}
}
