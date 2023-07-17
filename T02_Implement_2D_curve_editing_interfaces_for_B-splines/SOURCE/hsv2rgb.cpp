
#include <math.h>
#include <iostream>

//
// Convertes a color specification given in hsv parameters to
// the equivalent specification in rgb parameters. As input,
// h is in the rage 0 to 360, s and v in the range 0 to 1
//
void
HSV2RGB(double hsv[3], double rgb[3])
{
	double h = hsv[0];
	double s = hsv[1];
	double v = hsv[2];

	if (h < 0 || h >= 360) h -= floor(h / 360) * 360; // h in [0,360)

	h /= 60;					// convert h to be in [0, 6]

	int i = (int)h;				// integer part
	double f = h - i;			// fractional part

	double p1 = v * (1 - s);
	double p2 = v * (1 - s * f);
	double p3 = v * (1 - s * (1 - f));

	switch (i)
	{
	case 0: rgb[0] = v; rgb[1] = p3; rgb[2] = p1; break;
	case 1: rgb[0] = p2; rgb[1] = v; rgb[2] = p1; break;
	case 2: rgb[0] = p1; rgb[1] = v; rgb[2] = p3; break;
	case 3: rgb[0] = p1; rgb[1] = p2; rgb[2] = v; break;
	case 4: rgb[0] = p3; rgb[1] = p1; rgb[2] = v; break;
	case 5: rgb[0] = v; rgb[1] = p1; rgb[2] = p2; break;
	default: std::cerr << "Critical error in h of HSV2RGB()" << std::endl; break;
	}
}

void HSV2RGB(float hsv[3], float rgb[3])
{
	float h = hsv[0];
	float s = hsv[1];
	float v = hsv[2];

	if (h < 0 || h >= 360) h -= floor(h / 360) * 360; // h in [0,360)

	h /= 60;					// convert h to be in [0, 6]
	
	int i = (int)h;				// fractional part
	double f = h - i;			// integer part

	double p1 = v * (1 - s);
	double p2 = v * (1 - s * f);
	double p3 = v * (1 - s * (1 - f));

	switch (i)
	{
	case 0: rgb[0] = v;  rgb[1] = p3; rgb[2] = p1; break;
	case 1: rgb[0] = p2; rgb[1] = v;  rgb[2] = p1; break;
	case 2: rgb[0] = p1; rgb[1] = v;  rgb[2] = p3; break;
	case 3: rgb[0] = p1; rgb[1] = p2; rgb[2] = v; break;
	case 4: rgb[0] = p3; rgb[1] = p1; rgb[2] = v; break;
	case 5: rgb[0] = v;  rgb[1] = p1; rgb[2] = p2; break;
	default: std::cerr << "Critical error in h of HSV2RGB()" << std::endl; break;
	}
	
}