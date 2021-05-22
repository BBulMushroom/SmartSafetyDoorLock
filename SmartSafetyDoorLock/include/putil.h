/**
	PORT operation utility

	v0.1
		- bit operation added
		- port macro added.

	@project	TLib
	@author	tchan@TSoft
	@date	2014/01/01
*/
#ifndef	__PORT_UTIL_H__
#define __PORT_UTIL_H__

#define	_SET(p,b)			(p |=  (1<<b))
#define	_CLEAR(p,b)			(p &= ~(1<<b))
#define	_CLR(p,b)			(p &= ~(1<<b))
#define	_XOR(p,b)			(p ^=  (1<<b))


#define	BIT_SET(p,b)			(p |=  (1<<b))
#define	BIT_CLR(p,b)			(p &= ~(1<<b))
#define	BIT_XOR(p,b)			(p ^=  (1<<b))

#define	BIT_VALUE(data, b)	(data & (1<<b))


#define	IS_BIT_SET(data, b)	((data & (1<<b)) != 0)
#define	IS_BIT_CLR(data, b)	((data & (1<<b)) == 0)

/**
	ex. DDRD |= 0xF0;		//0 ~ 3 bit output

	PORT_DIR(DDRD, 0xF0);
*/
#define	PORT_DIR(p, value)	(p = value)


/**
	Just describe port with name and pin bit #.


	example.
		PORT(C, 0);			PORTC bit 0
*/
#define	PORT(a, b)		a, b

/**
	ex. DDRD |= 0xF0;		//0 ~ 3 bit output

	PORT_DIR_IN(PORT(D, 0))
*/

#define DIR_BIT_SET(p, b)	BIT_SET(DDR##p, b)
#define DIR_BIT_CLR(p, b)	BIT_CLR(DDR##p, b)


#define PORT_BIT_SET(p, b)	BIT_SET(PORT##p, b)
#define PORT_BIT_CLR(p, b)	BIT_CLR(PORT##p, b)
#define PORT_BIT_XOR(p, b)  BIT_XOR(PORT##p, b)

#define	PORT_BIT_VALUE(p, b)	BIT_VALUE(PIN##p, b)


//#define PORT_DIR_IN(port)	DDRD &= portPORT(D, 0))
/**
	PORT_DIR_IN(D, 0)

	DDRD &= (1<<0);
*/
//#define PORT_DIR_IN(p, b)	(DDR##p &= 3)
//#define PORT_DIR_IN(p, b)	BIT_CLR(DDR##p, b)


#define PORT_DIR_IN(port)	DIR_BIT_CLR(port)
#define PORT_DIR_OUT(port)	DIR_BIT_SET(port)


#define PORT_SET(port)	PORT_BIT_SET(port)
#define PORT_CLR(port)	PORT_BIT_CLR(port)
#define PORT_XOR(port)  PORT_BIT_XOR(port)

#define PORT_VALUE(port)	PORT_BIT_VALUE(port)

//#define PORT_DIR_IN(port)	DIR_SET(port)
//#define PORT_DIR_OUT(port)	DIR_CLR(port)


#define IS_PORT_BIT_SET(p, b)	IS_BIT_SET(PIN##p, b)
#define IS_PORT_BIT_CLR(p, b)	IS_BIT_CLR(PIN##p, b)

#define	IS_PORT_SET(port)	IS_PORT_BIT_SET(port)
#define	IS_PORT_CLR(port)	IS_PORT_BIT_CLR(port)


#endif	//__PORT_UTIL_H__
