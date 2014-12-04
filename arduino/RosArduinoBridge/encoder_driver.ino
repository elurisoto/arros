/* *************************************************************
	 Encoder definitions
	 
	 Add a "#if defined" block to this file to include support for
	 a particular encoder board or library. Then add the appropriate
	 #define near the top of the main ROSArduinoBridge.ino file.
	 
	 ************************************************************ */
#ifndef ENCODER_DRIVER
#define ENCODER_DRIVER
#ifdef USE_BASE
#if defined(SEN0038)
	#include "Encoders.h"
	extern Encoders enc;

	long readEncoder(int i){
		if (i == LEFT) return enc.getCountsM1();
		else return enc.getCountsM2();
	}

	void resetEncoder(int i){
		if (i == LEFT) enc.resetM1();
		else enc.resetM2();
	}

	/* Wrap the encoder reset function */
	void resetEncoders() {
		resetEncoder(LEFT);
		resetEncoder(RIGHT);
	}

#else
	#error A encoder driver must be selected!
#endif



#endif
#endif

