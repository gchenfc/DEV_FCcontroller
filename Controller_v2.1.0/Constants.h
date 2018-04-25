#ifndef Constants_h
#define Constants_h

// technical PWM stuff
#define TPM_C 48000000            // core clock, for calculation only
#define PWM_FREQ 100000            //  PWM frequency [Hz]
#define MODULO (TPM_C / PWM_FREQ) // calculation the modulo for FTM0
// less technical for non-FTM0
#define PWM_SPEED 50000
#define PWM_RES 12
#define MAXPWM 4096

// PID
#define KpPower 0.003
#define KiPower 0.003
#define KdPower -0.000005
#define KpCurrent .02
#define KiCurrent 0.001
#define KdCurrent -0.0001
#define Kp KpPower
#define Ki KiPower
#define Kd KdPower

#define MAXDESPOWER 100

// FC thermistor divider
#define TEMP_RES 1200
// 12V voltage divider
#define STABLE12_MULT 10

// short circuit codes
#define SC_OFF 0
#define SC_RAMPUP 1
#define SC_HOLD 2
#define SC_RAMPDOWN 3
#define SC_RECOV 4

#define DIP_PULLUP 0

#endif

