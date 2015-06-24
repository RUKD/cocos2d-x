//
//  Wrapper
//  SIMD
//
//  Created by RUKD on 15/1/13.
//  Copyright (c) 2015 Myst. All rights reserved.
//

#ifndef SIMD_Wrapper_h
#define SIMD_Wrapper_h

#include "SIMD.h"

namespace Myst
{
    static const Vector128 RNG_ExpMask(0x3F800000, 0x3F800000, 0x3F800000, 0x3F800000);
    static const Vector128 cVectorNegativeOne(-1.0f,-1.0f,-1.0f,-1.0f);
    static const Vector128 cVectorOne(1.0f,1.0f,1.0f,1.0f);
    static const Vector128 cVectorZero(0.0f,0.0f,0.0f,0.0f);
    static const Vector128 cVectorHalf(0.5f,0.5f,0.5f,0.5f);
    const Vector128 cVector255(255.0f);
    
    const Myst::Vector128 cVectorBoundaryMasks[4] = {
        Myst::Vector128((int)0xFFFFFFFF, (int)0xFFFFFFFF, (int)0xFFFFFFFF, (int)0xFFFFFFFF),
        Myst::Vector128(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF),
        Myst::Vector128(0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF),
        Myst::Vector128(0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF),
    };
    
    const Myst::Vector128 cVectorBoundaryMasksFlip[4] = {
        Myst::Vector128((int)0x00000000, (int)0x00000000, (int)0x00000000, (int)0x00000000),
        Myst::Vector128(0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000),
        Myst::Vector128(0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000),
        Myst::Vector128(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000),
    };
    
    const Myst::Vector128 cVector4DivPi(1.2732395447352f);
    const Myst::Vector128 cVectorMinus4DivPiPi(-0.40528473456f);
    const Myst::Vector128 cVectorSinFactor(0.225f);
    const Myst::Vector128 cVectorPi(3.14159265359f);
    const Myst::Vector128 cVectorHalfPi(1.57079632679f);
    const Myst::Vector128 cVectorMinusPi(-3.14159265359f);
    const Myst::Vector128 cVector2Pi(6.28318530718f);
    
    const Myst::Vector128 cVectorSign((int)0x80000000);
    
    
#define TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS)\
    {\
        const size_t head = range.start & 3;\
        const size_t tail = range.end & 3;\
        size_t i = range.start >> 2;\
        const size_t end = range.end >> 2;\
        \
        if (i == end) {\
            Vector128 boundaryMask = VectorAndNot(cVectorBoundaryMasks[tail],cVectorBoundaryMasks[head]);\
            BOUNDARY_STATEMENTS(i, boundaryMask);\
        }\
        else {\
            if (head) {\
                BOUNDARY_STATEMENTS(i, cVectorBoundaryMasks[head]);\
                i++;\
            }\
            \
            for (; i < end; i++) {\
                NORMAL_STATEMENTS(i);\
            }\
            \
            if (tail) {\
                BOUNDARY_STATEMENTS(i, cVectorBoundaryMasksFlip[tail]);\
            }\
        }\
    }
    
    
    
    
    //init operations
    
    inline Vector128 VectorZero() {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_setzero_ps();
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vdupq_n_f32(0.0f);
#else
        return Vector128(0.0f);
#endif
        
    }
    
    inline Vector128 VectorZeroI()
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_setzero_si128();
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vdupq_n_u32(0);
#else
        return Vector128((int)0);
#endif
        
    }
    
    inline Vector128 VectorSet(float x, float y, float z, float w) {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_set_ps(x, y, z, w);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        float f[4] = {x,y,z,w};
        return vld1q_f32(f);
#else
        return Vector128(x,y,z,w);
#endif
        
    }
    
    inline Vector128 VectorSet1(float f) {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_set_ps1(f);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vdupq_n_f32(f);
#else
        return Vector128(f,f,f,f);
#endif
    }
    
    inline void VectorStore(float* v,const Vector128& a)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        _mm_store_ps(v, a.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        vst1q_f32(v,a.v);
#else
        v[0] = a.f[0];
        v[1] = a.f[1];
        v[2] = a.f[2];
        v[3] = a.f[3];
#endif
    }
    
    //mul
    inline Vector128 I32VectorMult(const Vector128& a,const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        Vector128 tmp1 = _mm_mul_epu32(a.vi,b.vi); /* mul 2,0*/
        Vector128 tmp2 = _mm_mul_epu32( _mm_srli_si128(a.vi,4), _mm_srli_si128(b.vi,4)); /* mul 3,1 */
        return _mm_unpacklo_epi32(_mm_shuffle_epi32(tmp1.vi, _MM_SHUFFLE (0,0,2,0)), _mm_shuffle_epi32(tmp2.vi, _MM_SHUFFLE (0,0,2,0)));
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vmulq_u32(a.vu,b.vu);
#else
        return Vector128(a.i[0]*b.i[0],a.i[1]*b.i[1],a.i[2]*b.i[2],a.i[3]*b.i[3]);
#endif
    }
    
    inline Vector128 VectorMul(const Vector128& a , const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_mul_ps(a.v,b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vmulq_f32(a.v,b.v);
#else
        return Vector128(a.f[0]*b.f[0],a.f[1]*b.f[1],a.f[2]*b.f[2],a.f[3]*b.f[3]);
#endif
        //return _mm_mul_ps(a.v,b.v);
    }
    
    inline Vector128 VectorInverseSqrt(const Vector128& a)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_rsqrt_ps(a.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vrsqrteq_f32(a.v);
#else
        return Vector128(1.0f/sqrtf(a.f[0]),1.0f/sqrtf(a.f[1]),1.0f/sqrtf(a.f[2]),1.0f/sqrtf(a.f[3]));
#endif
        
    }
    
    inline Vector128 VectorDiv(const Vector128& a, const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_div_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        Vector128 t = vrecpeq_f32(b.v);
        return vmulq_f32(a.v,t.v);
#else
        return Vector128(a.f[0]/b.f[0],a.f[1]/b.f[1],a.f[2]/b.f[2],a.f[3]/b.f[3]);
#endif
    }

    inline Vector128 VectorOr(const Vector128& a , const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_or_ps(a.v,b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vorrq_u32(a.vi,b.vi);
#else
        return Vector128(a.i[0]|b.i[0],a.i[1]|b.i[1],a.i[2]|b.i[2],a.i[3]|b.i[3]);
#endif
    }
    
    inline Vector128 VectorEqI(const Vector128& a, const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cmpeq_epi32(a.vi, b.vi);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vceqq_u32(a.vu,b.vu);
#else
        return Vector128::Int128(
                         a.i[0]==b.i[0]?0xFFFFFFFF:0,
                         a.i[1]==b.i[1]?0xFFFFFFFF:0,
                         a.i[2]==b.i[2]?0xFFFFFFFF:0,
                         a.i[3]==b.i[3]?0xFFFFFFFF:0);
#endif
        
    }
    
    inline Vector128 VectorShiftRightI(const Vector128& a, int count)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_srli_epi32(a.vi,count);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        Vector128 c = vdupq_n_s32(-count);
        return vshlq_u32( a.vu,c.vi );
#else
        return Vector128::Int128(
                         a.i[0]>>count,
                         a.i[1]>>count,
                         a.i[2]>>count,
                         a.i[3]>>count
                         );
#endif
    }
    
    inline Vector128 VectorShiftLeftI(const Vector128& a, int count)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_slli_epi32(a.vi,count);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        Vector128 c = vdupq_n_s32(count);
        return vshlq_u32( a.vu,c.vi );
#else
        return Vector128::Int128(
                         a.i[0]<<count,
                         a.i[1]<<count,
                         a.i[2]<<count,
                         a.i[3]<<count
                         );
#endif
        
    }

    
    inline Vector128 VectorAndNot(const Vector128& a , const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_andnot_ps(a.v,b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        Vector128 t = vmvnq_u32(a.vi);
        return vandq_u32(t.vi,b.vi);
#else
        return Vector128::Int128(
                         (~a.i[0]&b.i[0]),
                         (~a.i[1]&b.i[1]),
                         (~a.i[2]&b.i[2]),
                         (~a.i[3]&b.i[3]));
#endif
    }
    
    inline Vector128 VectorAnd(const Vector128& a, const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_and_ps(a.v,b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vandq_u32(a.vi,b.vi);
#else
        return Vector128(
                         a.i[0]&b.i[0],
                         a.i[1]&b.i[1],
                         a.i[2]&b.i[2],
                         a.i[3]&b.i[3]);
#endif
    }
    
    inline Vector128 VectorSub(const Vector128& a , const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_sub_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vsubq_f32(a.v,b.v);
#else
        return Vector128(
                         a.f[0]-b.f[0],
                         a.f[1]-b.f[1],
                         a.f[2]-b.f[2],
                         a.f[3]-b.f[3]);
#endif
        
    }
    
    inline Vector128 VectorGreater(const Vector128& a,const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cmpgt_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vcgtq_f32(a.v,b.v);
#else
        return Vector128::Int128(
                         a.i[0]>b.i[0]?0xFFFFFFFF:0,
                         a.i[1]>b.i[1]?0xFFFFFFFF:0,
                         a.i[2]>b.i[2]?0xFFFFFFFF:0,
                         a.i[3]>b.i[3]?0xFFFFFFFF:0);
#endif
    }
    
    inline Vector128 VectorSelect(const Vector128& a , const Vector128& b , const Vector128& mask)
    {
        Vector128 temp1 = VectorAndNot(mask,a);//_mm_andnot_ps(control, a);
        Vector128 temp2 = VectorAnd(b,mask);
        return  VectorOr(temp1, temp2);
    }
    
    inline void StreamSet(const Vector128& a,float* p)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        _mm_stream_ps(p, a.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        VectorStore(p, a);
#else
        p[0] = a.f[0];
        p[1] = a.f[1];
        p[2] = a.f[2];
        p[3] = a.f[3];
#endif
    }
    
    inline Vector128 VectorConvertI32_F32(const Vector128& a)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cvtepi32_ps(a.vi);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vcvtq_f32_s32(a.vi);
#else
        return Vector128(0);
#endif

    }
    
    inline Vector128 VectorConvertF32_I32(const Vector128& a)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cvttps_epi32(a.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vcvtq_s32_f32(a.v);
#else
        return Vector128(0);
#endif
        
    }
    
    inline void VectorModf(const Vector128& v,Vector128& frac,Vector128& intg)
    {
        Vector128 v0 = VectorZeroI();
        Vector128 v1 = VectorEqI(v0,v0);
        Vector128 ji = VectorShiftRightI(v1, 25);
        Vector128 j  = VectorShiftLeftI(ji, 23);
        intg =  VectorConvertF32_I32(v);
        Vector128 f  = VectorConvertI32_F32(intg);
        frac    = VectorSub(v,f);
//#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
//        __m128i v0 = _mm_setzero_si128();
//        __m128i v1 = _mm_cmpeq_epi32(v0, v0);
//        __m128i ji = _mm_srli_epi32(v1, 25);
//        Vector128 j = _mm_slli_epi32(ji, 23);
//        intg = _mm_cvttps_epi32(v.v);
//        __m128 f = _mm_cvtepi32_ps(intg.vi);
//        frac = _mm_sub_ps(v.v, f);
//#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
//        __m128i v0 = _mm_setzero_si128();
//        __m128i v1 = _mm_cmpeq_epi32(v0, v0);
//        __m128i ji = _mm_srli_epi32(v1, 25);
//        Vector128 j = _mm_slli_epi32(ji, 23);
//        intg = _mm_cvttps_epi32(v.v);
//        __m128 f = _mm_cvtepi32_ps(intg.vi);
//        frac = _mm_sub_ps(v.v, f);
//#else
//#endif
        
       
    }
    
//    inline Vector128 VectorFToI(const Vector128& v)
//    {
//#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
//
//        return _mm_cvttps_epi32(v.v);
//#else
//        return Vector128::Int128((int)v.f[0],(int)v.f[1],(int)v.f[2],(int)v.f[3]);
//#endif
//    }
    
    inline int VectorGetMask(const Vector128& v)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        
        return _mm_movemask_ps(v.v);
#else
        //return Vector128((int)v.v[0],(int)v.v[1],(int)v.v[2],(int)v.v[3]);
        return 0;
#endif
    }
    
    inline Vector128 VectorAdd(const Vector128& a,const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_add_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vaddq_f32(a.v,b.v);
#else
        return Vector128(
                         a.f[0]+b.f[0],
                         a.f[1]+b.f[1],
                         a.f[2]+b.f[2],
                         a.f[3]+b.f[3]);
#endif
        
    }

    inline Vector128 I32VectorAdd(const Vector128& a, const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_add_epi32(a.vi,b.vi);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vaddq_u32(a.vu,b.vu);
#else
        return Vector128(
                         a.i[0]+b.i[0],
                         a.i[1]+b.i[1],
                         a.i[2]+b.i[2],
                         a.i[3]+b.i[3]);
#endif
    }

    inline Vector128 I32ToFloat01(const Vector128& a)
    {
        Vector128 i = VectorShiftRightI(a, 9);
        i = VectorOr(i, RNG_ExpMask);
        return VectorAdd(i, cVectorNegativeOne);
//#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
//        Vector128 i = _mm_srli_epi32(a.v, 9);
//        i = VectorOr(i, RNG_ExpMask);
//        return VectorAdd(i, cVectorNegativeOne);
//#else
//        return Vector128(0.0f);
//#endif
    }
    
    inline Vector128 I32ToFloatRange(const Vector128& a,const Vector128& start , const Vector128& range)
    {

        Vector128 float01 = I32ToFloat01(a);
        float01 = VectorMul(float01,range);
        return VectorAdd(float01, start);
    }
    
    inline Vector128 VectorGreatorEq(const Vector128& a,const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cmpnlt_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vcgeq_f32(a.v,b.v);
#else
        return Vector128(0.0f);
#endif
    }
    
    inline Vector128 VectorLessEq(const Vector128& a, const Vector128& b)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        return _mm_cmpngt_ps(a.v, b.v);
#elif MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        return vcleq_f32(a.v,b.v);
#else
        return Vector128(0.0f);
#endif
    }
    
    inline Vector128 VectorAbs(const Vector128& a)
    {
        return VectorAndNot(cVectorSign, a);
//#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
//        return
//        return _mm_andnot_ps(cVectorSign.v, a.v);
//#else
//        return Vector128(0.0f);
//#endif
    }
    
    inline Vector128 VectorSin(const Vector128& a)
    {
        Vector128 mask = VectorGreatorEq(a, cVectorPi);
        mask = VectorAnd(mask, cVector2Pi);
        Vector128 value = VectorSub(a, mask);
        mask = VectorLessEq(value, cVectorMinusPi);
        mask = VectorAnd(mask, cVector2Pi);
        value = VectorAdd(value, mask);
        
        Vector128 valueAbs = VectorAbs(value);
        Vector128 m1 = VectorMul(valueAbs, cVectorMinus4DivPiPi);
        m1 = VectorAdd(m1, cVector4DivPi);
        
        Vector128 m_y = VectorMul(m1, value);
        
        valueAbs = VectorAbs(m_y);
        
        m1 = VectorMul(valueAbs, m_y);
        m1 = VectorSub(m1, m_y);
        m1 = VectorMul(m1, cVectorSinFactor);
        m_y = VectorAdd(m1, m_y);
        return m_y;
    }
    
    inline Vector128 VectorCos(const Vector128& a)
    {
        Vector128 v = VectorAdd(a, cVectorHalfPi);
        Vector128 mask = VectorGreatorEq(v, cVector2Pi);
        mask = VectorAnd(mask,cVector2Pi);
        v = VectorSub(v, mask);
        return VectorSin(v);
    }
    
    inline void VectorTranspose(Vector128& r0,Vector128& r1,Vector128& r2,Vector128& r3)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_SSE
        Vector128 t0 = _mm_unpacklo_ps(r0.v, r1.v);
        Vector128 t1 = _mm_unpacklo_ps(r2.v, r3.v);
        Vector128 t2 = _mm_unpackhi_ps(r0.v, r1.v);
        Vector128 t3 = _mm_unpackhi_ps(r2.v, r3.v);
        
        r0 = _mm_movelh_ps(t0.v, t1.v);
        r1 = _mm_movehl_ps(t1.v, t0.v);
        r2 = _mm_movelh_ps(t2.v, t3.v);
        r3 = _mm_movehl_ps(t3.v, t2.v);
#else
        
        
        
        return;
#endif
    }
    
    inline void VectorTransposeTo(Vector128*d,float*p)
    {
#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
        float32x4x4_t* a = (float32x4x4_t*)d;
        vst4q_f32(p, *a);
#endif
    }
    
    
}

#endif


