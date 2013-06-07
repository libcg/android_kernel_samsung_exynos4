#ifndef	_UNIDISPLAY_I2C_TS_H
#define	_UNIDISPLAY_I2C_TS_H

struct unidisplay_ts_platform_data {
	int (*init)(void);
	int (*reset)(void);
	int (*pin_state)(int);
};

#endif
