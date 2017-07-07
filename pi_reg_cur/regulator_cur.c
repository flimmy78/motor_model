#include <scilab/scicos_block4.h>
#include <time.h>

#include <math.h>

#define pi		3.14159265358979323846	/* pi */

#define r_IN(n, i) ((GetRealInPortPtrs(blk, n+1))[(i)]) 
#define r_OUT(n, i) ((GetRealOutPortPtrs(blk, n+1))[(i)])

//#define Ki 1
//#define Kp 0


struct pi_reg_state{
	double ki;
	double kp;
	double a;
	double y;	
};

double dot3(double *a, double *b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void abc_to_dq(double *abc, double *dq, double phi)
{
	double ct[3] = {cos(phi), cos(phi-2*pi/3), cos(phi+2*pi/3)};
	double st[3] = {sin(phi), sin(phi-2*pi/3), sin(phi+2*pi/3)};
	
	dq[0] = dot3(abc, ct);	
	dq[1] = -dot3(abc, st);
}

void dq_to_abc(double *abc, double *dq, double phi)
{
	abc[0] = dq[0]*cos(phi) - 		 dq[1]*sin(phi);	
	abc[1] = dq[0]*cos(phi-2*pi/3) - dq[1]*sin(phi-2*pi/3);
	abc[2] = dq[0]*cos(phi+2*pi/3) - dq[1]*sin(phi+2*pi/3);
}

void svpwm(double *abc, double *dq, double phi)
{
	double d = dq[0];
	double q = dq[1];
	double mag = sqrt(d*d + q*q);
	if(mag > 720) mag = 720;
	double thetta = 0.0;
	if(fabs(d) > 0.01){
		 thetta = (d>0)?atan(q/d):(atan(q/d)+pi);
	}
	else thetta = (q>0)?(pi/2):(-pi/2);
	
	phi = phi + thetta;
	
	if(phi<0) phi += 2*pi;
	else if(phi>2*pi) phi -= 2*pi;
		
	double r1;
	double r2;
	
	if(phi<pi/3){
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);	
		
		abc[0] = r1+r2;
		abc[1] = -r1+r2;
		abc[2] = -r1-r2;
	}
	else if(phi<2*pi/3){
		phi -= pi/3;
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);		
		
		abc[0] = r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1-r2;		
	}
	else if(phi<pi){
		phi -= 2*pi/3;
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);		
		
		abc[0] = -r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1+r2;
	}
	else if(phi<4*pi/3){
		phi -= 3*pi/3;
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);		
		
		abc[0] = -r1-r2;
		abc[1] = r1-r2;
		abc[2] = r1+r2;		
	}		
	else if(phi<5*pi/3){
		phi -= 4*pi/3;
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);
		
		abc[0] = -r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1+r2;	
	}			
	else if(phi<2*pi){
		phi -= 5*pi/3;
		double r1 = mag*sin(pi/3-phi);
		double r2 = mag*sin(phi);		
		
		abc[0] = +r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1-r2;
	}
	
}

void update(struct pi_reg_state *s, double e)
{
    s->a += s->ki*e;
	s->y = e*s->kp + s->a;	
}

void pi_reg_cur(scicos_block *blk, int flag)
{
	static struct pi_reg_state dreg = {0.0, 0.0, 0.0, 0.0};
	static struct pi_reg_state qreg = {0.0, 0.0, 0.0, 0.0};
	static double phi;
	double dq[2];
	long angle;
	   
    switch(flag)
    {
	case 1:		
		
		//phi = r_IN(4, 0)*2*pi/4096;
		dq[0] = dreg.y;
		dq[1] = qreg.y;
		// convert dq voltages to abc
		//dq_to_abc(&r_OUT(0, 0), dq, phi);
		svpwm(&r_OUT(0, 0), dq, phi);
			    
	break;
	case 2:
	
		dreg.ki = r_IN(2, 0);
		dreg.kp = r_IN(3, 0);
		
		qreg.ki = r_IN(2, 0);
		qreg.kp = r_IN(3, 0);		
		
		// get the motor electrical angle
		//angle = ((long)r_IN(4, 0) << 2) & (4096-1);
		//phi = angle*2*pi/4096;
				
		angle = ( (unsigned long)r_IN(4, 0) ) & (1024-1);
		phi = angle*2*pi/1024;
		
		//phi = r_IN(4, 0)*2*pi/4096;
		
		// convert abc currents to dq
		abc_to_dq(&r_IN(0, 0), dq, phi);

		update(&dreg, r_IN(1, 0) - dq[0]);
		update(&qreg, r_IN(1, 1) - dq[1]);

	break;
	case 4:
	    dreg.a = 0.0;
	    dreg.y = 0.0;
	    
	    qreg.a = 0.0;
	    qreg.y = 0.0;	    
	    
	break;
    }
}
