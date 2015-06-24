//
//  SIMD.h
//  SIMD
//
//  Created by RUKD on 15/1/9.
//  Copyright (c) 2015年 Myst. All rights reserved.
//

#ifndef SIMD_SIMD_h
#define SIMD_SIMD_h


#include <cmath>




#define MYST_SIMD_NAME "SSE"
#define MYST_SIMD_VERSION 50
#define USE_INTRINSICS 1



#define MYST_SIMD_PLATFORM_NONE     0
#define MYST_SIMD_PLATFORM_SSE      1
#define MYST_SIMD_PLATFORM_NEON     2

#if defined( __WIN32__ ) || defined( _WIN32 )						//windows
#   define MYST_SIMD_PLATFORM MYST_SIMD_PLATFORM_INTEL
#elif defined(__APPLE_CC__)
#   if __ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__ >= 30000 || __IPHONE_OS_VERSION_MIN_REQUIRED >= 30000
#       include "TargetConditionals.h"
#       if TARGET_IPHONE_SIMULATOR 
#           define MYST_SIMD_PLATFORM MYST_SIMD_PLATFORM_NONE
#       else
#           define MYST_SIMD_PLATFORM MYST_SIMD_PLATFORM_NEON			//ios
#       endif
#   else
#       define MYST_SIMD_PLATFORM MYST_SIMD_PLATFORM_SSE				//mac
#   endif
#else																//Linux
#   define MYST_SIMD_PLATFORM MYST_SIMD_PLATFORM_SSE
#endif


#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
#   include <immintrin.h>
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
#   include <arm_neon.h>
#endif


namespace Myst
{
    
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
    
    typedef __m128  F128Register;
    typedef __m128i I128Register;
    typedef __m128i U128Register;
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
    typedef float32x4_t  F128Register;
    typedef int32x4_t    I128Register;
    typedef uint32x4_t   U128Register;
#else
    struct F128Register
    {
        float t[4];
    };
    struct I128Register
    {
        int   t[4];
    };
    struct U128Register
    {
        unsigned int t[4];
    };
#endif

    
    struct Vector128
    {
        union
        {
            int     i[4];
            float   f[4];
            F128Register v;
            I128Register vi;
            U128Register vu;
        };

        explicit Vector128(int c=0)
        {
            i[0] = c;
            i[1] = c;
            i[2] = c;
            i[3] = c;
        }
        
        explicit Vector128(float c)
        {
            f[0] = c;
            f[1] = c;
            f[2] = c;
            f[3] = c;
        }
        
        static inline Vector128 Float128(float x,float y , float z,float w)
        {
            return Vector128(x,y,z,w);
        }
        
        static inline Vector128 Int128(int x,int y , int z , int w)
        {
            return Vector128(x,y,z,w);
        }

        
        Vector128(int x,int y , int z , int w)
        {
            i[0] = x;
            i[1] = y;
            i[2] = z;
            i[3] = w;
        }
        
        Vector128(float x, float y , float z, float w)
        {
            f[0] = x;
            f[1] = y;
            f[2] = z;
            f[3] = w;
        }

        Vector128(F128Register f128)
        {
            v = f128;
        }

        Vector128(I128Register i128)
        {
            vi = i128;
        }


        inline Vector128& operator=(F128Register f128 )
        {
            v = f128;
            return *this;
        }

        inline Vector128& operator=(I128Register i128)
        {
            vi = i128;
            return * this;
        }
        
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        Vector128(U128Register u128)
        {
            vu = u128;
        }
        
        inline Vector128& operator=(U128Register u128)
        {
            vu = u128;
            return *this;
        }
#endif
        

    };
    
    struct Matrix128x3
    {
        Vector128 x,y,z;
    };
    
    struct Matrix128x4
    {
        Vector128 x,y,z,w;
    };
    
}




#endif
