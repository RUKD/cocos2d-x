//
//  RNG.h
//  SIMD
//
//  Created by RUKD on 15/1/14.
//  Copyright (c) 2015年 Myst. All rights reserved.
//

#ifndef SIMD_RNG_h
#define SIMD_RNG_h

#include "SIMD.h"
#include "Wrapper.h"


namespace Myst
{
    class LCG
    {
    private:
        //static const Vector128 AddConst;
        //static const Vector128 MultConst;
        
    public:
        LCG(int seed)
        :mSeed(seed,seed,seed,seed)
        ,AddConst(2531011, 10395331, 13737667, 1)
        ,MultConst(214013, 17405, 214013, 69069)
        {
            
        }
        

        inline Vector128 Next()
        {
            mSeed = I32VectorAdd(I32VectorMult(MultConst, mSeed), AddConst);
            return mSeed;
        }

        inline Vector128 NextFloat()
        {
            return I32ToFloat01(Next());
        }
        
        inline Vector128 NextFloatRange(float start , float end)
        {
            Vector128 start128(start);
            Vector128 range128(end-start);
            return I32ToFloatRange(Next(), start128, range128);
        }
        
    protected:
        Vector128 mSeed;
        Vector128 AddConst;
        Vector128 MultConst;
    };
    
    //const Vector128 LCG::AddConst(2531011, 10395331, 13737667, 1);
    //const Vector128 LCG::MultConst(214013, 17405, 214013, 69069);
}


#endif
