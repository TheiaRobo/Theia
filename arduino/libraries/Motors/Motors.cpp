 /*
 * Motors.h
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Motors Class
 * 
 * Without indications, value are in International System of Units
 * 
 */

#include <Motors.h>

/* Constructor */
Motors::Motors(char dir_pin,char pwm_pin,char brk_pin,char cfb_pin)	{
_dir_pin = dir_pin ;
_pwm_pin = pwm_pin ;
_brk_pin = brk_pin ;
_cfb_pin = cfb_pin ;

pinMode(_dir_pin,OUTPUT);
pinMode(_pwm_pin,OUTPUT);
pinMode(_brk_pin,OUTPUT);

}

/* Destructor */
Motors::~Motors(void) {}



/**
 * Generic methods
 */


/*
 * Motors::Set_speed(int u)
 * Set the duty cycle of the PWM
 * parameter >
 * 		@ int u : duty cycle value, between -255 and 255. 
 *                        -255: (reverse maximal speed) 
 *                        0: (no motion)
 *                        255: (maximal speed)
 * 		
 * 	Assuming that Vcc is you power supply voltage, voltage V applied to the motor is 
 * 	V = Vcc*u/255
 * 	
 */
int Motors::Set_speed(int u)	{	
	digitalWrite(_cfb_pin, LOW);  // No brake
	if (u<0)  {
		//if(u>-40)	{u=0;}
    		digitalWrite(_dir_pin, LOW);
    		analogWrite(_pwm_pin,-u);
  	}
  	else {
		//if(u<40)	{u=0;}
    		digitalWrite(_dir_pin, HIGH);
    		analogWrite(_pwm_pin,u);
 	 }
	return 0;
}

/*
 * Motors::Speed_regulation
 * Control the speed of the motors
 * parameters >
 * 		@ float W : 		 desired speed, rad/s
 * 		@ float Te : 		 sampling period, in seconds
 * 		@ int encoder : 	 encoder value
 * 		@ int encoder_old :	 encoder old value
 */

int Motors::Speed_regulation(float W, float Te, int encoder, int encoder_old) {
	float u ;
	_speed_instruction = W ;	
	Read_speed(encoder, encoder_old, Te);
	_error = _speed_instruction - _speed;
	_int+= _error*Te;

	if(_int>_int_max) {_int = _int_max;}
	else if(_int<-_int_max)  {_int = -_int_max;}

	u = _k*_error+_ki*_int;
	
	//Threshold	
	if(u>5)	{u+=35;}
	else if(u<-5)	{u-=35;}

	if(u > 255) {u = 255;}
	else if(u < -255)  {u = -255;}

	Set_speed(u);

}


/*
 * Motors::Read_Speed
 * Returns the measured Speed, in rad/s
 * parameters >
 * 		@ float Te : 		 sampling period, in seconds
 * 		@ int encoder : 	 encoder value
 * 		@ int encoder_old :	 encoder old value
 */
float Motors::Read_speed(int encoder, int encoder_old, float Te)	{
	int dp;
	dp = encoder - encoder_old ;
	_speed = dp*2.*M_PI/1000./Te;
	return _speed;
}


/*
 * Motors::Read_Current
 * Returns the measured Current, in Amps
 */
float Motors::Read_current() {
	return analogRead(_cfb_pin)*5./1024.*1.65;
}


/*
 * Motors::Set_control_parameters
 * Set the values of the control parameters
 * parameters >
 * 		@ float K :				proportionnal gain
 * 		@ float KI : 	 		integral gain
 * 		@ int INT_MAX :	 		integral saturation value
 * 		@ int ticks_per_rev :	Encoders ticks per revolution
 */
int Motors::Set_control_parameters(int K, int KI, int INT_MAX, int ticks_per_rev) {
	_k = K ;
	_ki = KI ;
	_int_max = INT_MAX ;
	_ticks_per_rev = ticks_per_rev ;
return 0;
}

int Motors::Reset()	{
	_int = 0 ;
}


