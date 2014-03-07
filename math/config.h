#ifndef MATH_CONFIG_H
#define MATH_CONFIG_H

namespace Math
{
	#pragma pack(push,1)
	struct vec3
	{
		long double x,y,z;	
	};
	#pragma pack(pop)

	#pragma pack(push,1)
	struct mat3
	{
		long double _00,_01,_02,
					_10,_11,_12,
					_20,_21,_22;	
	};
	#pragma pack(pop)

	typedef long double 		LD;
	typedef long double* 		ptrLD;
	typedef long double** 		pptrLD;
	typedef long double* const 	cptrLD;

	typedef unsigned int 		UI;
	typedef const unsigned int 	CUI;
    typedef int 				I;
    typedef const int 			CI;

	static constexpr LD M_PI 	= 3.1415926535897932384626433832795;
	static constexpr LD M_2PI 	= 6.283185307179586476925286766559;
}

#endif  //MATH_CONFIG_H
