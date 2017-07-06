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

void update(struct pi_reg_state *s, double e)
{
    s->a += s->ki*e;
	s->y = e*s->kp + s->a;	
}

void pi_reg_speed(scicos_block *blk, int flag)
{
	static struct pi_reg_state reg = {0.0, 0.0, 0.0, 0.0};
	double Ki, Kp;
	double e;
	double phi;
	long enc, denc, denc1;
	static long enc1 = 0;
	static long enc2 = 0;
	double speed;
	double smp_rate = 3e3;
    
    switch(flag)
    {
	case 1:		
	    r_OUT(0, 0) = reg.y;
	break;
	case 2:
	
		smp_rate = r_IN(5, 0);
	
		enc = (long)r_IN(4, 0);		
		denc = enc-enc1;
		enc1 = enc;
		if(abs(denc) > 1000){
			if(denc < 0) denc += 4096;
			else denc -= 4096;
		}		
		speed = denc*(60/2/pi)*smp_rate*2*pi/4096;

/*
		enc = (long)r_IN(4, 0);
		denc = enc-enc1;
		denc1 = enc1-enc2;
		if(abs(denc) > 1000){
			if(denc < 0) denc += 4096;
			else denc -= 4096;
		}		
		if(abs(denc1) > 1000){
			if(denc1 < 0) denc1 += 4096;
			else denc1 -= 4096;
		}						
		enc2 = enc1;
		enc1 = enc;		
		
		speed = 1.5*denc - 0.5*denc1;
		speed *= (60/2/pi/4)*smp_rate*2*pi/4096;
*/		
	
		
		
		//e = r_IN(1, 0) - r_IN(0, 0);
		e = r_IN(1, 0) - speed;
		r_OUT(1, 0) = speed;

		reg.ki = r_IN(2, 0);
		reg.kp = r_IN(3, 0);

		update(&reg, e);

	break;
	case 4:
	    reg.a = 0.0;
	    reg.y = 0.0;
	    
	    enc1 = 0;
	    enc2 = 0;
	break;
    }
}
