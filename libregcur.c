#include <scilab/scicos_block4.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define pi		3.14159265358979323846	/* pi */
#define MY_PI 512

#define r_IN(n, i) ((GetRealInPortPtrs(blk, n+1))[(i)]) 
#define r_OUT(n, i) ((GetRealOutPortPtrs(blk, n+1))[(i)])

//#define USE_SVPWM

extern const int32_t cos_tb[1024];

//#define Ki 1
//#define Kp 0

struct pi_reg_state{
	int32_t ki;
	int32_t kp;
	int32_t a;
	int32_t y;	
};

inline int32_t mycos(int32_t a)
{
	return cos_tb[1023&a];
}

inline int32_t mysin(int32_t a)
{
	return cos_tb[1023&(a+3*MY_PI/2)];
}

inline void update(struct pi_reg_state *s, int32_t e)
{
    s->a += s->ki*e;
	s->y = e*s->kp + s->a;	

	//s->y = 1024*e + s->a;
	//s->a = s->y - 782*e;
}

inline int32_t dot3(int32_t *a, int32_t *b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline void abc_to_dq(int32_t *abc, int32_t *dq, int32_t angle)
{
	int32_t ct[3] = {	cos_tb[angle], 
						cos_tb[1023&(angle+(4*512)/3)], 
						cos_tb[1023&(angle+(2*512)/3)] };
	int32_t st[3] = {	cos_tb[1023&(3*512/2+angle)], 
						cos_tb[1023&(3*512/2+angle+(4*512)/3)], 
						cos_tb[1023&(3*512/2+angle+2*512/3)] };

	dq[0] = (dot3(abc, ct)) >> 10;	
	dq[1] = (-dot3(abc, st)) >> 10;
}

inline void dq_to_abc(int32_t *abc, int32_t *dq, int32_t angle)
{
	abc[0] = (dq[0]*cos_tb[angle] 				- dq[1]*cos_tb[1023&(angle+3*512/2)]) >> 20;
	abc[1] = (dq[0]*cos_tb[1023&(angle+4*512/3)] - dq[1]*cos_tb[1023&(angle+4*512/3+3*512/2)]) >> 20;
	abc[2] = (dq[0]*cos_tb[1023&(angle+2*512/3)] - dq[1]*cos_tb[1023&(angle+2*512/3+3*512/2)]) >> 20;
}

inline void cord_atan(int32_t *v, int32_t *ang, int32_t *mag)
{
	const int32_t AngTable[] = {128, 76, 40, 20, 10, 5, 3, 1};
	const int32_t kc[] = {724,  648, 628,  623,  623,  622,  622,  622};
	int32_t SumAngle = 0; 
	int i = 0;
	int x, y, x1, y1;
	int ns = 0;

	x = abs(v[0]);
	y = v[1];

	for(i = 0; i < 8; i++)
	{		
		ns++;
		
		x1 = x;
		y1 = y;
			
		if(y > 0){
			x = x1 + (y1 >> i); 
			y = y1 - (x1 >> i); 
			SumAngle = SumAngle + AngTable[i]; 
		}else{
			x = x1 - (y1 >> i); 
			y = y1 + (x1 >> i); 
			SumAngle = SumAngle - AngTable[i]; 
		}
		if(y == 0) break;
	}
	
	if(v[0] < 0) SumAngle = MY_PI-SumAngle;		
	if(SumAngle < 0) SumAngle += 2*MY_PI;	
	
	*ang = SumAngle;
	*mag = (kc[ns-1]*x)/1024;
}

inline void svpwm(int32_t *abc, int32_t *dq, int32_t phase)
{
	int32_t mag;
	int32_t ang;
	cord_atan(dq, &ang, &mag);
	
	mag /= 1024;
	int32_t phi = 1023&(phase + ang);

	if(phi<MY_PI/3){
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = r1+r2;
		abc[1] = -r1+r2;
		abc[2] = -r1-r2;
	}
	else if(phi<2*MY_PI/3){
		phi -= MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1-r2;		
	}
	else if(phi<MY_PI){
		phi -= 2*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = -r1-r2;
		abc[1] = r1+r2;
		abc[2] = -r1+r2;
	}
	else if(phi<4*MY_PI/3){
		phi -= 3*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = -r1-r2;
		abc[1] = r1-r2;
		abc[2] = r1+r2;		
	}		
	else if(phi<5*MY_PI/3){
		phi -= 4*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = -r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1+r2;	
	}			
	else if(phi<2*MY_PI){
		phi -= 5*MY_PI/3;
		int32_t r1 = mag*mysin(7*MY_PI/3-phi) >> 10;
		int32_t r2 = mag*mysin(phi) >> 10;
		
		abc[0] = +r1+r2;
		abc[1] = -r1-r2;
		abc[2] = r1-r2;
	}	
}

void pi_reg_cur(scicos_block *blk, int flag)
{
	static struct pi_reg_state dreg = {0, 0, 0, 0};
	static struct pi_reg_state qreg = {0, 0, 0, 0};
	static int32_t phase = 0;
	
	int32_t dq[2];	
	int32_t abc[3];
	
	int32_t ed;
	int32_t eq;

    switch(flag)
    {
	case 1:		
		// convert dq voltages to abc			
				
#ifdef USE_SVPWM

		dq[0] = dreg.y;
		dq[1] = qreg.y;				
		
		r_OUT(1, 0) = 0;
		r_OUT(1, 1) = 0;
		
		svpwm(abc, dq, phase);
		r_OUT(0, 0) = (double)abc[0];
		r_OUT(0, 1) = (double)abc[1];
		r_OUT(0, 2) = (double)abc[2];

#else
		dq[0] = dreg.y;
		dq[1] = qreg.y;
		dq_to_abc(abc, dq, phase);
		r_OUT(0, 0) = (double)abc[0];
		r_OUT(0, 1) = (double)abc[1];
		r_OUT(0, 2) = (double)abc[2];		
#endif							    
							    
	break;
	case 2:		
		
		dreg.ki = r_IN(2, 0)*1024;
		dreg.kp = r_IN(3, 0)*1024;
		
		qreg.ki = r_IN(2, 0)*1024;
		qreg.kp = r_IN(3, 0)*1024;

		// get the motor electrical angle
		phase = ( (uint32_t)r_IN(4, 0) ) & (1024-1);

		// get the currents from ADC
		abc[0] = (int32_t)r_IN(0, 0);
		abc[1] = (int32_t)r_IN(0, 1);
		abc[2] = (int32_t)r_IN(0, 2);
		// convert abc currents to dq
		abc_to_dq(abc, dq, phase);
		
		// get the errors
		ed = (int32_t)r_IN(1, 0) - dq[0];
		eq = (int32_t)r_IN(1, 1) - dq[1];
		
		// regulators do its work
		update( &dreg, ed );
		update( &qreg, eq );

	break;
	case 4:
		// some init here
	    dreg.a = 0;
	    dreg.y = 0;
	    
	    qreg.a = 0;
	    qreg.y = 0;	    
	    
	break;
    }
}
