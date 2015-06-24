//
//  CCParticleSystemQuadSIMD.cpp
//  cocos2d_libs
//
//  Created by RUKD on 15/3/30.
//
//


/****************************************************************************
 Copyright (c) 2008-2010 Ricardo Quesada
 Copyright (c) 2009      Leonardo Kasperavičius
 Copyright (c) 2010-2012 cocos2d-x.org
 Copyright (c) 2011      Zynga Inc.
 Copyright (c) 2013-2014 Chukong Technologies Inc.
 
 http://www.cocos2d-x.org
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/


#include "2d/CCParticleSystemQuadSIMD.h"
#include "SIMD/SIMD.h"
#include "SIMD/Wrapper.h"
#include "SIMD/RNG.h"

#include <algorithm>

#include "2d/CCSpriteFrame.h"
#include "2d/CCParticleBatchNode.h"
#include "renderer/CCTextureAtlas.h"
#include "renderer/ccGLStateCache.h"
#include "renderer/CCRenderer.h"
#include "base/CCDirector.h"
#include "base/CCEventType.h"
#include "base/CCConfiguration.h"
#include "base/CCEventListenerCustom.h"
#include "base/CCEventDispatcher.h"

#include "deprecated/CCString.h"
#include "base/CCProfiling.h"

NS_CC_BEGIN

using namespace Myst;

struct SizeRange
{
    size_t start;
    size_t end;
};

ParticleSystemQuadSIMD::ParticleSystemQuadSIMD()
:_quads(nullptr)
,_indices(nullptr)
,_VAOname(0)
{
    memset(_buffersVBO, 0, sizeof(_buffersVBO));
}



ParticleSystemQuadSIMD::~ParticleSystemQuadSIMD()
{
    if (nullptr == _batchNode)
    {
        CC_SAFE_FREE(_quads);
        CC_SAFE_FREE(_indices);
        glDeleteBuffers(2, &_buffersVBO[0]);
        if (Configuration::getInstance()->supportsShareableVAO())
        {
            glDeleteVertexArrays(1, &_VAOname);
            GL::bindVAO(0);
        }
    }
    
    mParticleData.release();
}

// implementation ParticleSystemQuadSIMD

ParticleSystemQuadSIMD * ParticleSystemQuadSIMD::create(const std::string& filename)
{
    ParticleSystemQuadSIMD *ret = new (std::nothrow) ParticleSystemQuadSIMD();
    if (ret && ret->initWithFile(filename))
    {
        ret->autorelease();
        return ret;
    }
    CC_SAFE_DELETE(ret);
    return ret;
}

ParticleSystemQuadSIMD * ParticleSystemQuadSIMD::createWithTotalParticles(int numberOfParticles) {
    ParticleSystemQuadSIMD *ret = new (std::nothrow) ParticleSystemQuadSIMD();
    if (ret && ret->initWithTotalParticles(numberOfParticles))
    {
        ret->autorelease();
        return ret;
    }
    CC_SAFE_DELETE(ret);
    return ret;
}

ParticleSystemQuadSIMD * ParticleSystemQuadSIMD::create(ValueMap &dictionary)
{
    ParticleSystemQuadSIMD *ret = new (std::nothrow) ParticleSystemQuadSIMD();
    if (ret && ret->initWithDictionary(dictionary))
    {
        ret->autorelease();
        return ret;
    }
    CC_SAFE_DELETE(ret);
    return ret;
}

//implementation ParticleSystemQuadSIMD
// overriding the init method
bool ParticleSystemQuadSIMD::initWithTotalParticles(int numberOfParticles)
{
    // base initialization
    _totalParticles = numberOfParticles;

    
    mParticleData.release();
    
    
    if( !mParticleData.init(_totalParticles) )
    {
        CCLOG("Particle system Quad Neon: not enough memory");
        this->release();
        return false;
    }
    _allocatedParticles = numberOfParticles;
    
    if (_batchNode)
    {
        for (int i = 0; i < _totalParticles; i++)
        {
            mParticleData.atlasIndex[i] = i;
        }
    }
    // default, active
    _isActive = true;
    
    // default blend function
    _blendFunc = BlendFunc::ALPHA_PREMULTIPLIED;
    
    // default movement type;
    _positionType = PositionType::FREE;
    
    // by default be in mode A:
    _emitterMode = Mode::GRAVITY;
    
    // default: modulate
    // FIXME:: not used
    //    colorModulate = YES;
    
    _isAutoRemoveOnFinish = false;
    
    // Optimization: compile updateParticle method
    //updateParticleSel = @selector(updateQuadWithParticle:newPosition:);
    //updateParticleImp = (CC_UPDATE_PARTICLE_IMP) [self methodForSelector:updateParticleSel];
    //for batchNode
    _transformSystemDirty = false;

    //if( ParticleSystem::initWithTotalParticles(numberOfParticles) )
        // allocating data space
        if( ! this->allocMemory() ) {
            this->release();
            return false;
        }
        
        initIndices();
        if (Configuration::getInstance()->supportsShareableVAO())
        {
            setupVBOandVAO();
        }
        else
        {
            setupVBO();
        }
        
        setGLProgramState(GLProgramState::getOrCreateWithGLProgramName(GLProgram::SHADER_NAME_POSITION_TEXTURE_COLOR_NO_MVP));
        
#if CC_ENABLE_CACHE_TEXTURE_DATA
        // Need to listen the event only when not use batchnode, because it will use VBO
        auto listener = EventListenerCustom::create(EVENT_RENDERER_RECREATED, CC_CALLBACK_1(ParticleSystemQuadSIMD::listenRendererRecreated, this));
        _eventDispatcher->addEventListenerWithSceneGraphPriority(listener, this);
#endif
        
        return true;
}

bool ParticleSystemQuadSIMD::addParticle()
{
    assert("0&&should never call");
    if (this->isFull())
    {
        return false;
    }
    
    initParticleByIndex(_particleCount);
    ++_particleCount;
    
    return true;
}

inline void updateValueInRange(float* v , SizeRange& range , float start , float endd, LCG& lcg)
{
    Vector128* xx = (Vector128*)v;
    Vector128 temp;
    Vector128 start128(start);
    Vector128 range128(endd - start);
    
    
#define NORMAL_STATEMENTS(i) \
    xx[i] = I32ToFloatRange(lcg.Next(), start128, range128);
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    temp = I32ToFloatRange(lcg.Next(), start128, range128);\
    xx[i] = VectorSelect(xx[i],temp,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}

inline void updateDeltaValueInRange(float* v,float* dv,float* t,SizeRange& range , float start,float endd,LCG& lcg)
{
    Vector128* vv = (Vector128*)v;
    Vector128* dvv = (Vector128*)dv;
    Vector128* tt = (Vector128*)t;
    Vector128 temp;
    Vector128 start128(start);
    Vector128 range128(endd - start);
    
    
#define NORMAL_STATEMENTS(i) \
    dvv[i] = VectorDiv( VectorSub( I32ToFloatRange(lcg.Next(), start128, range128) , vv[i] ) , tt[i] );
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    temp = VectorDiv( VectorSub( I32ToFloatRange(lcg.Next(), start128, range128) , vv[i] ) , tt[i] );\
    dvv[i] = VectorSelect(dvv[i],temp,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}

inline void setConstValue(float* v , float constV,SizeRange& range)
{
    Vector128* vv = (Vector128*)v;
    Vector128 constVV(constV);
    
#define NORMAL_STATEMENTS(i) \
    vv[i] = constVV;
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    vv[i] = VectorSelect(vv[i],constVV,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}

inline void updateDirValue(float* x,float*y,SizeRange& range , float startAngle,float endAngle , float startSpeed,float endSpeed,LCG& lcg)
{
//    float a = CC_DEGREES_TO_RADIANS( _angle + _angleVar * CCRANDOM_MINUS1_1() );
//    Vec2 v(cosf( a ), sinf( a ));
//    float s = modeA.speed + modeA.speedVar * CCRANDOM_MINUS1_1();
//    Vec2 dir = v * s;
//    mParticleData.modeA.dirX[i] = dir.x;//v * s ;
//    mParticleData.modeA.dirY[i] = dir.y;
//    mParticleData.rotation[i] = -CC_RADIANS_TO_DEGREES(dir.getAngle());
    
    Vector128* xx = (Vector128*)x;
    Vector128* yy = (Vector128*)y;
    Vector128 tempX,tempY;
    Vector128 angle128;
    Vector128 speed128;
    Vector128 cos128,sin128;
    Vector128 startAngle128(startAngle);
    Vector128 rangeAngle128(endAngle - startAngle);
    Vector128 startSpeed128(startSpeed);
    Vector128 endSpeed128(endSpeed - startSpeed);
    
    
#define NORMAL_STATEMENTS(i) \
    angle128 = I32ToFloatRange(lcg.Next(), startAngle128, rangeAngle128);\
    cos128   = VectorCos(angle128);\
    sin128   = VectorSin(angle128);\
    speed128 = I32ToFloatRange(lcg.Next(), startSpeed128, endSpeed128);\
    xx[i] = VectorMul(cos128 , speed128);\
    yy[i] = VectorMul(sin128 , speed128);
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    angle128 = I32ToFloatRange(lcg.Next(), startAngle128, rangeAngle128);\
    cos128   = VectorCos(angle128);\
    sin128   = VectorSin(angle128);\
    speed128 = I32ToFloatRange(lcg.Next(), startSpeed128, endSpeed128);\
    tempX = VectorMul(cos128 , speed128);\
    tempY = VectorMul(sin128 , speed128);\
    xx[i] = VectorSelect(xx[i],tempX,boundaryMask);\
    yy[i] = VectorSelect(yy[i],tempY,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}


bool ParticleSystemQuadSIMD::addParticles(int count)
{
    int start = _particleCount;
    _particleCount += count;
    SizeRange range;
    range.start = start;
    range.end   = _particleCount;
    
#define GeneralUpdate(v)\
    updateValueInRange(mParticleData.v,range,startV,endV,lcg);
    
#define GeneralDeltaUpdate(v,dv)\
    updateDeltaValueInRange(mParticleData.v,mParticleData.dv,mParticleData.timeToLive,range,startV,endV,lcg);
    
    static LCG lcg(17);
    
    float startV = MAX(0,_life - _lifeVar);
    float endV   = MAX(0,_life + _lifeVar);
    GeneralUpdate(timeToLive);
    
    startV = _sourcePosition.x - _posVar.x;
    endV   = _sourcePosition.x + _posVar.x;
    GeneralUpdate(posx);
    
    startV = _sourcePosition.y - _posVar.y;
    endV   = _sourcePosition.y + _posVar.y;
    GeneralUpdate(posy);
    
    //color
#define SET_COLOR(c,b,v)\
    startV  = clampf( b - v , 0 , 1);\
    endV    = clampf( b + v , 0 , 1);\
    GeneralUpdate(c);
    
    SET_COLOR(colorR, _startColor.r, _startColorVar.r);
    SET_COLOR(colorG, _startColor.g, _startColorVar.g);
    SET_COLOR(colorB, _startColor.b, _startColorVar.b);
    SET_COLOR(colorA, _startColor.a, _startColorVar.a);
    
    SET_COLOR(deltaColorR, _endColor.r, _endColorVar.r);
    SET_COLOR(deltaColorG, _endColor.g, _endColorVar.g);
    SET_COLOR(deltaColorB, _endColor.b, _endColorVar.b);
    SET_COLOR(deltaColorA, _endColor.a, _endColorVar.a);
    
#undef SET_COLOR
    
#define SET_DELTA_COLOR(c,dc,b,v)\
    startV  = clampf( b - v , 0 , 1);\
    endV    = clampf( b + v , 0 , 1);\
    updateDeltaValueInRange(mParticleData.c,mParticleData.dc,mParticleData.timeToLive,range,startV,endV,lcg);

    
    SET_DELTA_COLOR(colorR, deltaColorR,_endColor.r, _endColorVar.r);
    SET_DELTA_COLOR(colorG, deltaColorG,_endColor.g, _endColorVar.g);
    SET_DELTA_COLOR(colorB, deltaColorB,_endColor.b, _endColorVar.b);
    SET_DELTA_COLOR(colorA, deltaColorA,_endColor.a, _endColorVar.a);
    
#undef SET_COLOR
    //size
    startV  = MAX(0 , _startSize - _startSizeVar);
    endV    = MAX(0 , _startSize + _startSizeVar);
    GeneralUpdate(size);
    
    if (_endSize != START_SIZE_EQUAL_TO_END_SIZE)
    {
        startV  = MAX(0 , _endSize - _endSizeVar);
        endV    = MAX(0 , _endSize + _endSizeVar);
        GeneralDeltaUpdate(size,deltaSize);
    }else
    {
        setConstValue(mParticleData.deltaSize,0.0f,range);
    }
    
    // rotation
    startV = _startSpin - _startSpinVar;
    endV   = _startSpin + _startSpinVar;
    GeneralUpdate(rotation);
    
    startV = _endSpin - _endSpinVar;
    endV   = _endSpin + _endSpinVar;
    GeneralDeltaUpdate(rotation, deltaRotation);
    
    // position
    Vec2 pos = _position;
    if (_positionType == PositionType::FREE)
    {
        pos = this->convertToWorldSpace(Vec2::ZERO);
    }
    else if (_positionType == PositionType::RELATIVE)
    {
        pos = _position;
    }
    setConstValue(mParticleData.startPosX, pos.x, range);
    setConstValue(mParticleData.startPosY, pos.y, range);
    
    // Mode Gravity: A
    if (_emitterMode == Mode::GRAVITY)
    {
        
        // radial accel
        startV = modeA.radialAccel - modeA.radialAccelVar;
        endV   = modeA.radialAccel + modeA.radialAccelVar;
        GeneralUpdate(modeA.radialAccel);
        
        // tangential accel
        startV = modeA.tangentialAccel - modeA.tangentialAccelVar;
        endV   = modeA.tangentialAccel + modeA.tangentialAccelVar;
        GeneralUpdate(modeA.tangentialAccel);
        
        // rotation is dir
        
        if( modeA.rotationIsDir )
        {
            updateDirValue(mParticleData.modeA.dirX,mParticleData.modeA.dirY,range,
                           CC_DEGREES_TO_RADIANS(_angle-_angleVar) , CC_DEGREES_TO_RADIANS(_angle + _angleVar),
                           modeA.speed - modeA.speedVar , modeA.speed + modeA.speedVar,lcg);
//            for (int i = start; i < _particleCount; ++i)
//            {
//                float a = CC_DEGREES_TO_RADIANS( _angle + _angleVar * CCRANDOM_MINUS1_1() );
//                Vec2 v(cosf( a ), sinf( a ));
//                float s = modeA.speed + modeA.speedVar * CCRANDOM_MINUS1_1();
//                Vec2 dir = v * s;
//                mParticleData.modeA.dirX[i] = dir.x;//v * s ;
//                mParticleData.modeA.dirY[i] = dir.y;
//                mParticleData.rotation[i] = -CC_RADIANS_TO_DEGREES(dir.getAngle());
//            }
        }else
        {
            updateDirValue(mParticleData.modeA.dirX,mParticleData.modeA.dirY,range,
                           CC_DEGREES_TO_RADIANS(_angle-_angleVar) , CC_DEGREES_TO_RADIANS(_angle + _angleVar),
                           modeA.speed - modeA.speedVar , modeA.speed + modeA.speedVar,lcg);
//            for (int i = start; i < _particleCount; ++i)
//            {
//                float a = CC_DEGREES_TO_RADIANS( _angle + _angleVar * CCRANDOM_MINUS1_1() );
//                Vec2 v(cosf( a ), sinf( a ));
//                float s = modeA.speed + modeA.speedVar * CCRANDOM_MINUS1_1();
//                Vec2 dir = v * s;
//                mParticleData.modeA.dirX[i] = dir.x;//v * s ;
//                mParticleData.modeA.dirY[i] = dir.y;
//            }
        }
        
    }
    
    // Mode Radius: B
    else
    {
        assert(0&&"RUKD need check");
        //        // Set the default diameter of the particle from the source position
        //        float startRadius = modeB.startRadius + modeB.startRadiusVar * CCRANDOM_MINUS1_1();
        //        float endRadius = modeB.endRadius + modeB.endRadiusVar * CCRANDOM_MINUS1_1();
        //
        //        mParticleData.modeB.radius[index] = startRadius;
        //
        //        if (modeB.endRadius == START_RADIUS_EQUAL_TO_END_RADIUS)
        //        {
        //            mParticleData.modeB.deltaRadius[index] = 0;
        //        }
        //        else
        //        {
        //            mParticleData.modeB.deltaRadius[index] = (endRadius - startRadius) / timeToLive;
        //        }
        //
        //        mParticleData.modeB.angle[index] = a;
        //        mParticleData.modeB.degreesPerSecond[index] = CC_DEGREES_TO_RADIANS(modeB.rotatePerSecond + modeB.rotatePerSecondVar * CCRANDOM_MINUS1_1());
    }
    
    
    
    return true;
}

bool ParticleSystemQuadSIMD::initParticleByIndex(int index)
{
    assert(index < mParticleData.getMaxCount());
    // timeToLive
    // no negative life. prevent division by 0
    float timeToLive = _life + _lifeVar * CCRANDOM_MINUS1_1();
    timeToLive = MAX(0, timeToLive);
    mParticleData.timeToLive[index] = timeToLive;
    
    // position
    mParticleData.posx[index] = _sourcePosition.x + _posVar.x * CCRANDOM_MINUS1_1();
    
    mParticleData.posy[index] = _sourcePosition.y + _posVar.y * CCRANDOM_MINUS1_1();
    
    
    // Color
    Color4F start;
    start.r = clampf(_startColor.r + _startColorVar.r * CCRANDOM_MINUS1_1(), 0, 1);
    start.g = clampf(_startColor.g + _startColorVar.g * CCRANDOM_MINUS1_1(), 0, 1);
    start.b = clampf(_startColor.b + _startColorVar.b * CCRANDOM_MINUS1_1(), 0, 1);
    start.a = clampf(_startColor.a + _startColorVar.a * CCRANDOM_MINUS1_1(), 0, 1);
    
    Color4F end;
    end.r = clampf(_endColor.r + _endColorVar.r * CCRANDOM_MINUS1_1(), 0, 1);
    end.g = clampf(_endColor.g + _endColorVar.g * CCRANDOM_MINUS1_1(), 0, 1);
    end.b = clampf(_endColor.b + _endColorVar.b * CCRANDOM_MINUS1_1(), 0, 1);
    end.a = clampf(_endColor.a + _endColorVar.a * CCRANDOM_MINUS1_1(), 0, 1);
    
    mParticleData.colorR[index] = start.r;
    mParticleData.colorG[index] = start.g;
    mParticleData.colorB[index] = start.b;
    mParticleData.colorA[index] = start.a;
    
    mParticleData.deltaColorR[index] = (end.r - start.r) / timeToLive;
    mParticleData.deltaColorG[index] = (end.g - start.g) / timeToLive;
    mParticleData.deltaColorB[index] = (end.b - start.b) / timeToLive;
    mParticleData.deltaColorA[index] = (end.a - start.a) / timeToLive;
    
    // size
    //RUKD fuck it
    float startS = _startSize + _startSizeVar * CCRANDOM_MINUS1_1();
    startS = MAX(0, startS); // No negative value
    
    mParticleData.size[index] = startS;
    
    float deltaSize = 0;
    if (_endSize != START_SIZE_EQUAL_TO_END_SIZE)
    {
        float endS = _endSize + _endSizeVar * CCRANDOM_MINUS1_1();
        endS = MAX(0, endS); // No negative values
        deltaSize = (endS - startS) / timeToLive;
        
    }
    
    mParticleData.deltaSize[index] = deltaSize;
    
    // rotation
    float startA = _startSpin + _startSpinVar * CCRANDOM_MINUS1_1();
    float endA = _endSpin + _endSpinVar * CCRANDOM_MINUS1_1();
    mParticleData.rotation[index] = startA;
    mParticleData.deltaRotation[index] = (endA - startA) / timeToLive;
    
    // position
    Vec2 pos = _position;
    if (_positionType == PositionType::FREE)
    {
        pos = this->convertToWorldSpace(Vec2::ZERO);
    }
    else if (_positionType == PositionType::RELATIVE)
    {
        pos = _position;
    }
    mParticleData.startPosX[index] = pos.x;
    mParticleData.startPosY[index] = pos.y;
    
    // direction
    float a = CC_DEGREES_TO_RADIANS( _angle + _angleVar * CCRANDOM_MINUS1_1() );
    
    //RUKD fuck it
    // Mode Gravity: A
    if (_emitterMode == Mode::GRAVITY)
    {
        Vec2 v(cosf( a ), sinf( a ));
        float s = modeA.speed + modeA.speedVar * CCRANDOM_MINUS1_1();
        
        // direction
        Vec2 dir = v * s;
        mParticleData.modeA.dirX[index] = dir.x;//v * s ;
        mParticleData.modeA.dirY[index] = dir.y;
        
        // radial accel
        mParticleData.modeA.radialAccel[index] = modeA.radialAccel + modeA.radialAccelVar * CCRANDOM_MINUS1_1();
        
        
        // tangential accel
        mParticleData.modeA.tangentialAccel[index] = modeA.tangentialAccel + modeA.tangentialAccelVar * CCRANDOM_MINUS1_1();
        
        // rotation is dir
        if(modeA.rotationIsDir)
            mParticleData.rotation[index] = -CC_RADIANS_TO_DEGREES(dir.getAngle());
    }
    
    // Mode Radius: B
    else
    {
        // Set the default diameter of the particle from the source position
        float startRadius = modeB.startRadius + modeB.startRadiusVar * CCRANDOM_MINUS1_1();
        float endRadius = modeB.endRadius + modeB.endRadiusVar * CCRANDOM_MINUS1_1();
        
        mParticleData.modeB.radius[index] = startRadius;
        
        if (modeB.endRadius == START_RADIUS_EQUAL_TO_END_RADIUS)
        {
            mParticleData.modeB.deltaRadius[index] = 0;
        }
        else
        {
            mParticleData.modeB.deltaRadius[index] = (endRadius - startRadius) / timeToLive;
        }
        
        mParticleData.modeB.angle[index] = a;
        mParticleData.modeB.degreesPerSecond[index] = CC_DEGREES_TO_RADIANS(modeB.rotatePerSecond + modeB.rotatePerSecondVar * CCRANDOM_MINUS1_1());
    }
    return true;
}

void ParticleSystemQuadSIMD::resetSystem()
{
    _isActive = true;
    _elapsed = 0;
    
    for (int i = 0; i < _particleCount; ++i)
    {
        mParticleData.timeToLive[i] = 0;
    }
}

inline void updateLifeTime(float* lifeTime , SizeRange& range , float deltaTime)
{
    Vector128* rLife = (Vector128*)lifeTime;
    Vector128  dT(-deltaTime);
    Vector128  temp;
    
#define NORMAL_STATEMENTS(i) \
    rLife[i] = VectorAdd(rLife[i],dT);
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    temp = VectorAdd(rLife[i],dT);\
    rLife[i] = VectorSelect(rLife[i],temp,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
    
    
}

inline void updateAddDeltaMultTime(float* value,float* deltaValue,SizeRange& range , float deltaTime)
{
    Vector128* v = (Vector128*)value;
    Vector128* dv = (Vector128*)deltaValue;
    Vector128  dT(deltaTime);
    Vector128  temp;
    
#define NORMAL_STATEMENTS(i) \
    v[i] = VectorAdd(v[i],VectorMul(dv[i], dT));
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    temp = VectorAdd(v[i],VectorMul(dv[i], dT));\
    v[i] = VectorSelect(v[i],temp,boundaryMask);
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}

inline void updateModeADir(float* dx,float* dy,float* x,float* y,float* radialAcc,float*tangentialAcc,
                           float deltaTime,Vec2 gravity,float _yCoordFlipped,SizeRange& range)
{
//    {
//        Vec2 tmp, radial, tangential;
//        
//        radial.set(mParticleData.posx[i], mParticleData.posy[i]);// = Vec2::ZERO;
//        
//        // radial acceleration
//        if (radial.x || radial.y)
//        {
//            radial.normalize();
//        }else
//        {
//            radial = Vec2::ZERO;
//        }
//        tangential = radial;
//        radial = radial * mParticleData.modeA.radialAccel[i];
//        
//        // tangential acceleration
//        float newy = tangential.x;
//        tangential.x = -tangential.y;
//        tangential.y = newy;
//        tangential = tangential * mParticleData.modeA.tangentialAccel[i];
//        
//        // (gravity + radial + tangential) * dt
//        tmp = radial + tangential + modeA.gravity;
//        tmp = tmp * dt;
//        mParticleData.modeA.dirX[i] = mParticleData.modeA.dirX[i] + tmp.x;
//        mParticleData.modeA.dirY[i] = mParticleData.modeA.dirY[i] + tmp.y;
    
        Vector128* xx = (Vector128*)x;
        Vector128* yy = (Vector128*)y;
        Vector128* dXX = (Vector128*)dx;
        Vector128* dYY = (Vector128*)dy;
        Vector128* radialAccel = (Vector128*)radialAcc;
        Vector128* tangentialAccel = (Vector128*)tangentialAcc;
        Vector128 radialX,radialY;
        Vector128 lenSqr,lenInverseSqrt;
        Vector128 tangentialX,tangentialY;
        Vector128 dirDeltaX,dirDeltaY;
       // Vector128* dv = (Vector128*)deltaValue;
        Vector128  dT(deltaTime);
        Vector128  tempX,tempY;
        Vector128  compareMask;
        Vector128 gravityX(gravity.x);
        Vector128 gravityY(gravity.y);
        
        
#define NORMAL_STATEMENTS(i) \
        radialX = VectorMul(xx[i],xx[i]);\
        radialY = VectorMul(yy[i],yy[i]);\
        lenSqr  = VectorAdd(radialX,radialY);\
        compareMask = VectorGreater(lenSqr , cVectorZero);\
        lenSqr = VectorAnd(lenSqr , compareMask);\
        lenInverseSqrt = VectorInverseSqrt(lenSqr);\
        radialX = VectorMul(radialX,lenInverseSqrt);\
        radialX = VectorAnd(radialX, compareMask);\
        radialY = VectorMul(radialY,lenInverseSqrt);\
        radialY = VectorAnd(radialY, compareMask);\
        tangentialX = VectorMul( radialY , tangentialAccel[i] );\
        tangentialY = VectorMul( radialX , tangentialAccel[i] );\
        radialX = VectorMul(radialX ,radialAccel[i]);\
        radialY = VectorMul(radialY ,radialAccel[i]);\
        dirDeltaX = VectorSub(radialX , tangentialX);\
        dirDeltaY = VectorAdd(radialY , tangentialY);\
        dirDeltaX = VectorAdd(dirDeltaX , gravityX);\
        dirDeltaY = VectorAdd(dirDeltaY , gravityY);\
        dirDeltaX = VectorMul(dirDeltaX , dT);\
        dirDeltaY = VectorMul(dirDeltaY , dT);\
        dXX[i] = VectorAdd(dXX[i] , dirDeltaX);\
        dYY[i] = VectorAdd(dYY[i] , dirDeltaY);
        
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
        radialX = VectorMul(xx[i],xx[i]);\
        radialY = VectorMul(yy[i],yy[i]);\
        lenSqr  = VectorAdd(radialX,radialY);\
        compareMask = VectorGreater(lenSqr , cVectorZero);\
        lenSqr = VectorAnd(lenSqr , compareMask);\
        lenInverseSqrt = VectorInverseSqrt(lenSqr);\
        radialX = VectorMul(radialX,lenInverseSqrt);\
        radialX = VectorAnd(radialX, compareMask);\
        radialY = VectorMul(radialY,lenInverseSqrt);\
        radialY = VectorAnd(radialY, compareMask);\
        tangentialX = VectorMul( radialY , tangentialAccel[i] );\
        tangentialY = VectorMul( radialX , tangentialAccel[i] );\
        radialX = VectorMul(radialX ,radialAccel[i]);\
        radialY = VectorMul(radialY ,radialAccel[i]);\
        dirDeltaX = VectorSub(radialX , tangentialX);\
        dirDeltaY = VectorAdd(radialY , tangentialY);\
        dirDeltaX = VectorAdd(dirDeltaX , gravityX);\
        dirDeltaY = VectorAdd(dirDeltaY , gravityY);\
        dirDeltaX = VectorMul(dirDeltaX , dT);\
        dirDeltaY = VectorMul(dirDeltaY , dT);\
        tempX = VectorAdd(dXX[i] , dirDeltaX);\
        tempY = VectorAdd(dYY[i] , dirDeltaY);\
        dXX[i] = VectorSelect(dXX[i],tempX,boundaryMask);\
        dYY[i] = VectorSelect(dYY[i],tempY,boundaryMask);
        
        
        TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
        
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
    
//    tmp.x = mParticleData.modeA.dirX[i] * dt * _yCoordFlipped;
//    tmp.y = mParticleData.modeA.dirY[i] * dt * _yCoordFlipped;
//    mParticleData.posx[i] = mParticleData.posx[i] + tmp.x;
//    mParticleData.posy[i] = mParticleData.posy[i] + tmp.y;
    
    Vector128 dTT(deltaTime*_yCoordFlipped);
    
#define NORMAL_STATEMENTS(i) \
    xx[i] = VectorAdd(xx[i],VectorMul(dXX[i], dTT));\
    yy[i] = VectorAdd(yy[i],VectorMul(dYY[i], dTT));
    
#define BOUNDARY_STATEMENTS(i, boundaryMask) \
    tempX = VectorAdd(xx[i],VectorMul(dXX[i], dTT));\
    tempY = VectorAdd(yy[i],VectorMul(dYY[i], dTT));\
    dXX[i] = VectorSelect(dXX[i],tempX,boundaryMask);\
    dYY[i] = VectorSelect(dYY[i],tempY,boundaryMask);
    
    
    TAG_PRESTIGE_ITERATERANGE(range, NORMAL_STATEMENTS, BOUNDARY_STATEMENTS);
    
#undef NORMAL_STATEMENTS
#undef BOUNDARY_STATEMENTS
}


void ParticleSystemQuadSIMD::update(float dt)
{
   
    CC_PROFILER_START_CATEGORY(kProfilerCategoryParticles , "CCParticleSystemSIMD - update");
    if( !_isActive )
        return;
    
    if (_isActive && _emissionRate)
    {
        
        _elapsed += dt;
        if (_duration != -1 && _duration < _elapsed)
        {
            this->stopSystem();
        }
    }
    int emitCount = 0;
    if (_isActive && _emissionRate)
    {
        _emitCounter += _emissionRate * dt;
        emitCount = _emitCounter;
        _emitCounter -= emitCount;
        emitCount = MIN(emitCount, _totalParticles - _particleCount);
        
//        float rate = 1.0f / _emissionRate;
//        //issue #1201, prevent bursts of particles, due to too high emitCounter
//        if (_particleCount < _totalParticles)
//        {
//            _emitCounter += dt;
//        }
//        
//        while (_particleCount < _totalParticles && _emitCounter > rate)
//        {
//            this->addParticle();
//            _emitCounter -= rate;
//        }
    }else
    {
        return;
    }

    
    {
        
        
        SizeRange range;
        range.start = 0;
        range.end = _particleCount;
        
        updateLifeTime(mParticleData.timeToLive, range, dt);

        
        for (int i = 0 ; i < _particleCount; ++i)
        {
            if( mParticleData.timeToLive[i] <= 0.0f )
            {
                mParticleData.swapParticle(i, _particleCount-1);
                --_particleCount;
            }
        }
        
        //add particle
        addParticles(emitCount);
        
        if( _particleCount == 0 && _isAutoRemoveOnFinish )
        {
            this->unscheduleUpdate();
            _parent->removeChild(this, true);
            return;
        }
        
        range.end = _particleCount;
        
        if( _emitterMode == Mode::GRAVITY )
        {
            static bool useNENO = true;
            if( useNENO )
            {
                updateModeADir(mParticleData.modeA.dirX,mParticleData.modeA.dirY,mParticleData.posx,mParticleData.posy,
                               mParticleData.modeA.radialAccel,mParticleData.modeA.tangentialAccel,
                               dt,modeA.gravity,_yCoordFlipped,range);
            }else
            {
            for (int i = 0 ; i < _particleCount; ++i)
            {
                Vec2 tmp, radial, tangential;
                
                radial.set(mParticleData.posx[i], mParticleData.posy[i]);// = Vec2::ZERO;
                
                // radial acceleration
                if (radial.x || radial.y)
                {
                    radial.normalize();
                }else
                {
                    radial = Vec2::ZERO;
                }
                tangential = radial;
                radial = radial * mParticleData.modeA.radialAccel[i];
                
                // tangential acceleration
                float newy = tangential.x;
                tangential.x = -tangential.y;
                tangential.y = newy;
                tangential = tangential * mParticleData.modeA.tangentialAccel[i];
                
                // (gravity + radial + tangential) * dt
                tmp = radial + tangential + modeA.gravity;
                tmp = tmp * dt;
                mParticleData.modeA.dirX[i] = mParticleData.modeA.dirX[i] + tmp.x;
                mParticleData.modeA.dirY[i] = mParticleData.modeA.dirY[i] + tmp.y;
                
                
                // this is cocos2d-x v3.0
                //                    if (_configName.length()>0 && _yCoordFlipped != -1)
                
                // this is cocos2d-x v3.0
                tmp.x = mParticleData.modeA.dirX[i] * dt * _yCoordFlipped;
                tmp.y = mParticleData.modeA.dirY[i] * dt * _yCoordFlipped;
                mParticleData.posx[i] = mParticleData.posx[i] + tmp.x;
                mParticleData.posy[i] = mParticleData.posy[i] + tmp.y;
            }
            }

            
        }else
        {
            for (int i = 0; i < _particleCount; ++i)
            {
                mParticleData.modeB.angle[i] += mParticleData.modeB.degreesPerSecond[i] * dt;
            }
            
            for (int i = 0; i < _particleCount; ++i)
            {
                mParticleData.modeB.radius[i] += mParticleData.modeB.deltaRadius[i] * dt;
            }
            
            for (int i = 0; i < _particleCount; ++i)
            {
                mParticleData.posx[i] = - cosf(mParticleData.modeB.angle[i]) * mParticleData.modeB.radius[i];
            }
            for (int i = 0; i < _particleCount; ++i)
            {
                mParticleData.posy[i] = - sinf(mParticleData.modeB.angle[i]) * mParticleData.modeB.radius[i]* _yCoordFlipped;
            }
        }
        
        
        //color r,g,b,a
        updateAddDeltaMultTime(mParticleData.colorR,mParticleData.deltaColorR,range,dt);
        updateAddDeltaMultTime(mParticleData.colorG,mParticleData.deltaColorG,range,dt);
        updateAddDeltaMultTime(mParticleData.colorB,mParticleData.deltaColorB,range,dt);
        updateAddDeltaMultTime(mParticleData.colorA,mParticleData.deltaColorA,range,dt);
        
        //size
        updateAddDeltaMultTime(mParticleData.size,mParticleData.deltaSize,range,dt);
        
        //angle
        updateAddDeltaMultTime(mParticleData.rotation,mParticleData.deltaRotation,range,dt);
        
        
   
        
        //update vertex data
        updateVertexData();
        
        _transformSystemDirty = false;
    }
    CC_PROFILER_STOP_CATEGORY(kProfilerCategoryParticles , "CCParticleSystemSIMD - update");
    // only update gl buffer when visible
    if (_visible && ! _batchNode)
    {
        //CC_PROFILER_START_CATEGORY(kProfilerCategoryParticles , "CCParticleSystemNeonSuperPostStep - update");
        postStep();
        //CC_PROFILER_STOP_CATEGORY(kProfilerCategoryParticles , "CCParticleSystemNeonSuperPostStep - update");
    }
    
   
}

inline void updateColorWithParticle(V3F_C4B_T2F_Quad *quad,float r,float g,float b,float a)
{
    
}

inline void updatePosWithParticle(V3F_C4B_T2F_Quad *quad, const Vec2& newPosition,float size,float rotation)
{
    // vertices
    GLfloat size_2 = size/2;
    if (rotation)
    {
        GLfloat x1 = -size_2;
        GLfloat y1 = -size_2;
        
        GLfloat x2 = size_2;
        GLfloat y2 = size_2;
        GLfloat x = newPosition.x;
        GLfloat y = newPosition.y;
        
        GLfloat r = (GLfloat)-CC_DEGREES_TO_RADIANS(rotation);
        GLfloat cr = cosf(r);
        GLfloat sr = sinf(r);
        GLfloat ax = x1 * cr - y1 * sr + x;
        GLfloat ay = x1 * sr + y1 * cr + y;
        GLfloat bx = x2 * cr - y1 * sr + x;
        GLfloat by = x2 * sr + y1 * cr + y;
        GLfloat cx = x2 * cr - y2 * sr + x;
        GLfloat cy = x2 * sr + y2 * cr + y;
        GLfloat dx = x1 * cr - y2 * sr + x;
        GLfloat dy = x1 * sr + y2 * cr + y;
        
        // bottom-left
        quad->bl.vertices.x = ax;
        quad->bl.vertices.y = ay;
        
        // bottom-right vertex:
        quad->br.vertices.x = bx;
        quad->br.vertices.y = by;
        
        // top-left vertex:
        quad->tl.vertices.x = dx;
        quad->tl.vertices.y = dy;
        
        // top-right vertex:
        quad->tr.vertices.x = cx;
        quad->tr.vertices.y = cy;
    }
    else
    {
        // bottom-left vertex:
        quad->bl.vertices.x = newPosition.x - size_2;
        quad->bl.vertices.y = newPosition.y - size_2;
        
        // bottom-right vertex:
        quad->br.vertices.x = newPosition.x + size_2;
        quad->br.vertices.y = newPosition.y - size_2;
        
        // top-left vertex:
        quad->tl.vertices.x = newPosition.x - size_2;
        quad->tl.vertices.y = newPosition.y + size_2;
        
        // top-right vertex:
        quad->tr.vertices.x = newPosition.x + size_2;
        quad->tr.vertices.y = newPosition.y + size_2;
    }
}

inline void writeToBufferWithOutRotate(float* x, float*y ,float* startX,float* startY,
                                       float* r,float* g,float* b,float*a , float* s,
                                       V3F_C4B_T2F* start,Mat4& worldToNodeTM,SizeRange& range)
{
    Vector128* xx = (Vector128*)x;
    Vector128* yy = (Vector128*)y;
    Vector128* sk = (Vector128*)s;
    Vector128* startXX = (Vector128*)startX;
    Vector128* startYY = (Vector128*)startY;
    Vector128 newPosX;
    Vector128 newPosY;
    Vector128 sizeScale;
    Vector128 m0_128 = Vector128(worldToNodeTM.m[0]);
    Vector128 m4_128 = Vector128(worldToNodeTM.m[4]);
    Vector128 m12_128= Vector128(worldToNodeTM.m[12]);
    Vector128 m1_128 = Vector128(worldToNodeTM.m[1]);
    Vector128 m5_128 = Vector128(worldToNodeTM.m[5]);
    Vector128 m13_128 = Vector128(worldToNodeTM.m[13]);
    Vector128 outData[4][4];
//#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
    float theData[4][16];
//#endif
    Vector128 outColor;
    Vector128 constColor((int)0xFFFFFFFF);
    Vector128* rr = (Vector128*)r;
    Vector128* gg = (Vector128*)g;
    Vector128* bb = (Vector128*)b;
    Vector128* aa = (Vector128*)a;
    
    Vector128 multiGen[4][2] =
    {
        {
            Vector128(-1.0f),
            Vector128(-1.0f),
        },
        {
            Vector128(1.0f),
            Vector128(-1.0f),
        },
        {
            Vector128(-1.0f),
            Vector128(1.0f),
 
        },
        {
            Vector128(1.0f),
            Vector128(1.0f),
        },
    };

    //Vector128* pp = (Vector128*)pos;
//    Vec3 p2(*startX,*startY,0);
//    worldToNodeTM.transformPoint(&p2);
//    Vec2 newPos(*x,*y);
//    p2 = p1 - p2;
//    newPos = newPos - Vec2(p2.x,p2.y) + _pos;

#define COMPUTE_NEWPOS\
    newPosX = VectorAdd( VectorMul(startXX[i],m0_128),VectorMul(startYY[i],m4_128) );\
    newPosX = VectorAdd( newPosX , m12_128);\
    newPosX = VectorAdd( xx[i] , newPosX );\
    newPosY = VectorAdd( VectorMul(startXX[i],m1_128),VectorMul(startYY[i],m5_128) );\
    newPosY = VectorAdd( newPosY , m13_128);\
    newPosY = VectorAdd( yy[i] , newPosY );
  
#define COMPUTE_ROW(row,column,t)\
    outData[row][column] = VectorAdd( t , VectorMul( sizeScale,multiGen[row][column] ) );
    
#define COMPUTE_POS(index)\
    COMPUTE_ROW(index, 0, newPosX);\
    COMPUTE_ROW(index, 1, newPosY);\
    outData[index][2] = cVectorZero;\
    outData[index][3] = outColor;\
    VectorTransposeTo(outData[index],theData[index]);
    
#define COMPUTE_SIZE\
    sizeScale = VectorMul( sk[i] , cVectorHalf );
    
#define COMPUTE_COLOR\
    outColor = VectorConvertF32_I32(VectorMul(rr[i],cVector255));\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(gg[i],cVector255)) , 8 ) );\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(bb[i],cVector255)) , 16 ) );\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(aa[i],cVector255)) , 24 ) );
    
    
#define OUTPUT_POS(index)\
    memcpy(start,&theData[0][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[1][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[2][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[3][index*4],sizeof(float)*4);start++;
    
    
#define NORMAL_STATEMENTS(i) \
    COMPUTE_COLOR\
    COMPUTE_NEWPOS\
    COMPUTE_SIZE\
    COMPUTE_POS(0);\
    COMPUTE_POS(1);\
    COMPUTE_POS(2);\
    COMPUTE_POS(3);\
    OUTPUT_POS(0);\
    OUTPUT_POS(1);\
    OUTPUT_POS(2);\
    OUTPUT_POS(3);
    
#define COMPTUE_DATA\
    COMPUTE_COLOR\
    COMPUTE_NEWPOS\
    COMPUTE_SIZE\
    COMPUTE_POS(0);\
    COMPUTE_POS(1);\
    COMPUTE_POS(2);\
    COMPUTE_POS(3);
    
    
    const size_t head = range.start & 3;
    const size_t tail = range.end & 3;
    size_t i = range.start >> 2;
    const size_t end = range.end >> 2;
    assert(head==0&&"invalid head");
    
    
    for (; i < end; i++)
    {
        NORMAL_STATEMENTS(i);
    }
    
    if (tail)
    {
        COMPTUE_DATA;
        for (int t = 0; t < tail; ++t)
        {
            OUTPUT_POS(t);
        }
    }
    
#undef COMPUTE_ROW
#undef COMPUTE_POS
#undef COMPUTE_COLOR
#undef OUTPUT_POS
#undef NORMAL_STATEMENTS
#undef COMPTUE_DATA
}

/*inline void writeToBufferWithRotate(float* x, float*y ,float* startX,float* startY,
                                       float* r,float* g,float* b,float*a , float* scale,float* rotate,
                                       V3F_C4B_T2F_Quad* start,Mat4& worldToNodeTM,SizeRange& range)
{
    Vector128* xx = (Vector128*)x;
    Vector128* yy = (Vector128*)y;
    Vector128* sk = (Vector128*)scale;
    Vector128* rotate128 = (Vector128*)rotate;
    Vector128* startXX = (Vector128*)startX;
    Vector128* startYY = (Vector128*)startY;
    Vector128 newPosX;
    Vector128 newPosY;
    Vector128 sizeScale;
    Vector128 m0_128 = Vector128(worldToNodeTM.m[0]);
    Vector128 m4_128 = Vector128(worldToNodeTM.m[4]);
    Vector128 m12_128= Vector128(worldToNodeTM.m[12]);
    Vector128 m1_128 = Vector128(worldToNodeTM.m[1]);
    Vector128 m5_128 = Vector128(worldToNodeTM.m[5]);
    Vector128 m13_128 = Vector128(worldToNodeTM.m[13]);
    Vector128 outData[4][4];
    //#if MYST_SIMD_PLATFORM == MYST_SIMD_PLATFORM_NEON
    float theData[4][16];
    //#endif
    Vector128 outColor;
    Vector128 constColor((int)0xFFFFFFFF);
    Vector128* rr = (Vector128*)r;
    Vector128* gg = (Vector128*)g;
    Vector128* bb = (Vector128*)b;
    Vector128* aa = (Vector128*)a;
    Vector128 c,s;
    
    Vector128 multiGen[4][2] =
    {
        {
            Vector128(-1.0f),
            Vector128(-1.0f),
        },
        {
            Vector128(1.0f),
            Vector128(-1.0f),
        },
        {
            Vector128(-1.0f),
            Vector128(1.0f),
            
        },
        {
            Vector128(1.0f),
            Vector128(1.0f),
        },
    };
    
    //Vector128* pp = (Vector128*)pos;
    
    
#define COMPUTE_NEWPOS\
newPosX = VectorAdd( VectorMul(startXX[i],m0_128),VectorMul(startYY[i],m4_128) );\
newPosX = VectorAdd( newPosX , m12_128);\
newPosY = VectorAdd( VectorMul(startXX[i],m1_128),VectorMul(startYY[i],m5_128) );\
newPosY = VectorAdd( newPosY , m13_128);
    
#define COMPUTE_ROW(row,column,t)\
    outData[row][column] = VectorAdd( t , VectorMul( sizeScale,multiGen[row][column] ) );
    
#define COMPUTE_POS(index)\
    COMPUTE_ROW(index, 0, newPosX);\
    COMPUTE_ROW(index, 1, newPosY);\
    outData[index][2] = cVectorZero;\
    outData[index][3] = outColor;\
    VectorTransposeTo(outData[index],theData[index]);
    
#define COMPUTE_SIZE\
sizeScale = VectorMul( sk[i] , cVectorHalf );
    
#define COMPUTE_COLOR\
    outColor = VectorConvertF32_I32(VectorMul(rr[i],cVector255));\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(gg[i],cVector255)) , 8 ) );\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(bb[i],cVector255)) , 16 ) );\
    outColor = VectorOr( outColor , VectorShiftLeftI( VectorConvertF32_I32(VectorMul(aa[i],cVector255)) , 24 ) );
    
    
#define OUTPUT_POS(index)\
    memcpy(start,&theData[0][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[1][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[2][index*4],sizeof(float)*4);start++;\
    memcpy(start,&theData[3][index*4],sizeof(float)*4);start++;
    
    
#define NORMAL_STATEMENTS(i) \
    COMPUTE_COLOR\
    COMPUTE_NEWPOS\
    COMPUTE_SIZE\
    COMPUTE_POS(0);\
    COMPUTE_POS(1);\
    COMPUTE_POS(2);\
    COMPUTE_POS(3);\
    OUTPUT_POS(0);\
    OUTPUT_POS(1);\
    OUTPUT_POS(2);\
    OUTPUT_POS(3);
    
#define COMPTUE_DATA\
    COMPUTE_COLOR\
    COMPUTE_NEWPOS\
    COMPUTE_SIZE\
    COMPUTE_POS(0);\
    COMPUTE_POS(1);\
    COMPUTE_POS(2);\
    COMPUTE_POS(3);
    
    
    const size_t head = range.start & 3;
    const size_t tail = range.end & 3;
    size_t i = range.start >> 2;
    const size_t end = range.end >> 2;
    assert(head==0&&"invalid head");
    
    
    for (; i < end; i++)
    {
        NORMAL_STATEMENTS(i);
    }
    
    if (tail)
    {
        COMPTUE_DATA;
        for (int t = 0; t < tail; ++t)
        {
            OUTPUT_POS(t);
        }
    }
    
#undef COMPUTE_ROW
#undef COMPUTE_POS
#undef COMPUTE_COLOR
#undef OUTPUT_POS
#undef NORMAL_STATEMENTS
#undef COMPTUE_DATA
}*/



void ParticleSystemQuadSIMD::updateVertexData()
{
    Mat4 worldToNodeTM = getWorldToNodeTransform();
    Vec2 currentPosition = Vec2::ZERO;
    
    if (_positionType == PositionType::FREE)
    {
        currentPosition = this->convertToWorldSpace(Vec2::ZERO);
    }
    else if (_positionType == PositionType::RELATIVE)
    {
        currentPosition = _position;
    }
    
    Vec3 p1(currentPosition.x,currentPosition.y,0);
    worldToNodeTM.transformPoint(&p1);
    
    
    V3F_C4B_T2F_Quad *startQuad;
    Vec2 _pos = Vec2::ZERO;
    assert(_batchNode == NULL);
    if (_batchNode)
    {
        V3F_C4B_T2F_Quad *batchQuads = _batchNode->getTextureAtlas()->getQuads();
        startQuad = &(batchQuads[_atlasIndex]);
        _pos = _position;
    }
    else
    {
        startQuad = &(_quads[0]);
    }
    
    
    printf("particle count %d\n",_particleCount);
    
    SizeRange range;
    range.start = 0;
    range.end = _particleCount;
    
    if( _positionType == PositionType::FREE )
    {
//        if( _haveRotate )
//        {
//            float* startX = mParticleData.startPosX;
//            float* startY = mParticleData.startPosY;
//            float* x = mParticleData.posx;
//            float* y = mParticleData.posy;
//            float* s = mParticleData.size;
//            float* r = mParticleData.rotation;
//            V3F_C4B_T2F_Quad* quadStart = startQuad;
//            for (int i = 0 ; i < _particleCount; ++i,++startX,++startY,++x,++y,++quadStart,++s,++r)
//            {
//                Vec3 p2(*startX,*startY,0);
//                worldToNodeTM.transformPoint(&p2);
//                Vec2 newPos(*x,*y);
//                p2 = p1 - p2;
//                newPos = newPos - Vec2(p2.x,p2.y) + _pos;
//                updatePosWithParticle(quadStart,newPos,*s,*r);
//            }
//        }else
        {
            writeToBufferWithOutRotate(mParticleData.posx, mParticleData.posy, mParticleData.startPosX, mParticleData.startPosY,
                                       mParticleData.colorR, mParticleData.colorG, mParticleData.colorB, mParticleData.colorA,
                                       mParticleData.size, (V3F_C4B_T2F*)startQuad, worldToNodeTM, range);
        }
        

    }else if( _positionType == PositionType::RELATIVE )
    {
        float* startX = mParticleData.startPosX;
        float* startY = mParticleData.startPosY;
        float* x = mParticleData.posx;
        float* y = mParticleData.posy;
        float* s = mParticleData.size;
        float* r = mParticleData.rotation;
        V3F_C4B_T2F_Quad* quadStart = startQuad;
        for (int i = 0 ; i < _particleCount; ++i,++startX,++startY,++x,++y,++quadStart,++s,++r)
        {
            Vec2 newPos(*x,*y);
            newPos.x = *x - (currentPosition.x - *startX);
            newPos.y = *y - (currentPosition.y - *startY);
            newPos += _pos;
            updatePosWithParticle(quadStart,newPos,*s,*r);
        }
    }else
    {
        float* startX = mParticleData.startPosX;
        float* startY = mParticleData.startPosY;
        float* x = mParticleData.posx;
        float* y = mParticleData.posy;
        float* s = mParticleData.size;
        float* r = mParticleData.rotation;
        V3F_C4B_T2F_Quad* quadStart = startQuad;
        for (int i = 0 ; i < _particleCount; ++i,++startX,++startY,++x,++y,++quadStart,++s,++r)
        {
            Vec2 newPos(*x+_pos.x,*y+_pos.y);
            updatePosWithParticle(quadStart,newPos,*s,*r);
        }
    }
    
    //set color
//    if(_opacityModifyRGB)
//    {
//        V3F_C4B_T2F_Quad* quad = startQuad;
//        float* r = mParticleData.colorR;
//        float* g = mParticleData.colorG;
//        float* b = mParticleData.colorB;
//        float* a = mParticleData.colorA;
//        
//        for (int i = 0; i < _particleCount; ++i,++quad,++r,++g,++b,++a)
//        {
//            Color4B color( *r * *a * 255, *g * *a * 255, *b * *a * 255, *a * 255);
//            quad->bl.colors = color;
//            quad->br.colors = color;
//            quad->tl.colors = color;
//            quad->tr.colors = color;
//        }
//    }else
//    {
//        V3F_C4B_T2F_Quad* quad = startQuad;
//        float* r = mParticleData.colorR;
//        float* g = mParticleData.colorG;
//        float* b = mParticleData.colorB;
//        float* a = mParticleData.colorA;
//        
//        for (int i = 0; i < _particleCount; ++i,++quad,++r,++g,++b,++a)
//        {
//            Color4B color( *r * 255, *g * 255, *b * 255, *a * 255);
//            quad->bl.colors = color;
//            quad->br.colors = color;
//            quad->tl.colors = color;
//            quad->tr.colors = color;
//        }
//    }

    

}

// pointRect should be in Texture coordinates, not pixel coordinates
void ParticleSystemQuadSIMD::initTexCoordsWithRect(const Rect& pointRect)
{
    // convert to Tex coords
    
    Rect rect = Rect(
                     pointRect.origin.x * CC_CONTENT_SCALE_FACTOR(),
                     pointRect.origin.y * CC_CONTENT_SCALE_FACTOR(),
                     pointRect.size.width * CC_CONTENT_SCALE_FACTOR(),
                     pointRect.size.height * CC_CONTENT_SCALE_FACTOR());
    
    GLfloat wide = (GLfloat) pointRect.size.width;
    GLfloat high = (GLfloat) pointRect.size.height;
    
    if (_texture)
    {
        wide = (GLfloat)_texture->getPixelsWide();
        high = (GLfloat)_texture->getPixelsHigh();
    }
    
#if CC_FIX_ARTIFACTS_BY_STRECHING_TEXEL
    GLfloat left = (rect.origin.x*2+1) / (wide*2);
    GLfloat bottom = (rect.origin.y*2+1) / (high*2);
    GLfloat right = left + (rect.size.width*2-2) / (wide*2);
    GLfloat top = bottom + (rect.size.height*2-2) / (high*2);
#else
    GLfloat left = rect.origin.x / wide;
    GLfloat bottom = rect.origin.y / high;
    GLfloat right = left + rect.size.width / wide;
    GLfloat top = bottom + rect.size.height / high;
#endif // ! CC_FIX_ARTIFACTS_BY_STRECHING_TEXEL
    
    // Important. Texture in cocos2d are inverted, so the Y component should be inverted
    std::swap(top, bottom);
    
    V3F_C4B_T2F_Quad *quads = nullptr;
    unsigned int start = 0, end = 0;
    if (_batchNode)
    {
        quads = _batchNode->getTextureAtlas()->getQuads();
        start = _atlasIndex;
        end = _atlasIndex + _totalParticles;
    }
    else
    {
        quads = _quads;
        start = 0;
        end = _totalParticles;
    }
    
    for(unsigned int i=start; i<end; i++)
    {
        // bottom-left vertex:
        quads[i].bl.texCoords.u = left;
        quads[i].bl.texCoords.v = bottom;
        // bottom-right vertex:
        quads[i].br.texCoords.u = right;
        quads[i].br.texCoords.v = bottom;
        // top-left vertex:
        quads[i].tl.texCoords.u = left;
        quads[i].tl.texCoords.v = top;
        // top-right vertex:
        quads[i].tr.texCoords.u = right;
        quads[i].tr.texCoords.v = top;
    }
}

void ParticleSystemQuadSIMD::updateTexCoords()
{
    if (_texture)
    {
        const Size& s = _texture->getContentSize();
        initTexCoordsWithRect(Rect(0, 0, s.width, s.height));
    }
}

void ParticleSystemQuadSIMD::setTextureWithRect(Texture2D *texture, const Rect& rect)
{
    // Only update the texture if is different from the current one
    if( !_texture || texture->getName() != _texture->getName() )
    {
        ParticleSystem::setTexture(texture);
    }
    
    this->initTexCoordsWithRect(rect);
}

void ParticleSystemQuadSIMD::setTexture(Texture2D* texture)
{
    const Size& s = texture->getContentSize();
    this->setTextureWithRect(texture, Rect(0, 0, s.width, s.height));
}

void ParticleSystemQuadSIMD::setDisplayFrame(SpriteFrame *spriteFrame)
{
    CCASSERT(spriteFrame->getOffsetInPixels().equals(Vec2::ZERO),
             "QuadParticle only supports SpriteFrames with no offsets");
    
    // update texture before updating texture rect
    if ( !_texture || spriteFrame->getTexture()->getName() != _texture->getName())
    {
        this->setTexture(spriteFrame->getTexture());
    }
}

void ParticleSystemQuadSIMD::initIndices()
{
    for(int i = 0; i < _totalParticles; ++i)
    {
        const unsigned int i6 = i*6;
        const unsigned int i4 = i*4;
        _indices[i6+0] = (GLushort) i4+0;
        _indices[i6+1] = (GLushort) i4+1;
        _indices[i6+2] = (GLushort) i4+2;
        
        _indices[i6+5] = (GLushort) i4+1;
        _indices[i6+4] = (GLushort) i4+2;
        _indices[i6+3] = (GLushort) i4+3;
    }
}

void ParticleSystemQuadSIMD::updateQuadWithParticle(tParticle* particle, const Vec2& newPosition)
{
    assert(0&&"no need call");
//    V3F_C4B_T2F_Quad *quad;
//    
//    if (_batchNode)
//    {
//        V3F_C4B_T2F_Quad *batchQuads = _batchNode->getTextureAtlas()->getQuads();
//        quad = &(batchQuads[_atlasIndex+particle->atlasIndex]);
//    }
//    else
//    {
//        quad = &(_quads[_particleIdx]);
//    }
//    Color4B color = (_opacityModifyRGB)
//    ? Color4B( particle->color.r*particle->color.a*255, particle->color.g*particle->color.a*255, particle->color.b*particle->color.a*255, particle->color.a*255)
//    : Color4B( particle->color.r*255, particle->color.g*255, particle->color.b*255, particle->color.a*255);
//    
//    quad->bl.colors = color;
//    quad->br.colors = color;
//    quad->tl.colors = color;
//    quad->tr.colors = color;
//    
//    // vertices
//    GLfloat size_2 = particle->size/2;
//    if (particle->rotation)
//    {
//        GLfloat x1 = -size_2;
//        GLfloat y1 = -size_2;
//        
//        GLfloat x2 = size_2;
//        GLfloat y2 = size_2;
//        GLfloat x = newPosition.x;
//        GLfloat y = newPosition.y;
//        
//        GLfloat r = (GLfloat)-CC_DEGREES_TO_RADIANS(particle->rotation);
//        GLfloat cr = cosf(r);
//        GLfloat sr = sinf(r);
//        GLfloat ax = x1 * cr - y1 * sr + x;
//        GLfloat ay = x1 * sr + y1 * cr + y;
//        GLfloat bx = x2 * cr - y1 * sr + x;
//        GLfloat by = x2 * sr + y1 * cr + y;
//        GLfloat cx = x2 * cr - y2 * sr + x;
//        GLfloat cy = x2 * sr + y2 * cr + y;
//        GLfloat dx = x1 * cr - y2 * sr + x;
//        GLfloat dy = x1 * sr + y2 * cr + y;
//        
//        // bottom-left
//        quad->bl.vertices.x = ax;
//        quad->bl.vertices.y = ay;
//        
//        // bottom-right vertex:
//        quad->br.vertices.x = bx;
//        quad->br.vertices.y = by;
//        
//        // top-left vertex:
//        quad->tl.vertices.x = dx;
//        quad->tl.vertices.y = dy;
//        
//        // top-right vertex:
//        quad->tr.vertices.x = cx;
//        quad->tr.vertices.y = cy;
//    }
//    else
//    {
//        // bottom-left vertex:
//        quad->bl.vertices.x = newPosition.x - size_2;
//        quad->bl.vertices.y = newPosition.y - size_2;
//        
//        // bottom-right vertex:
//        quad->br.vertices.x = newPosition.x + size_2;
//        quad->br.vertices.y = newPosition.y - size_2;
//        
//        // top-left vertex:
//        quad->tl.vertices.x = newPosition.x - size_2;
//        quad->tl.vertices.y = newPosition.y + size_2;
//        
//        // top-right vertex:
//        quad->tr.vertices.x = newPosition.x + size_2;
//        quad->tr.vertices.y = newPosition.y + size_2;
//    }
}
void ParticleSystemQuadSIMD::postStep()
{
    glBindBuffer(GL_ARRAY_BUFFER, _buffersVBO[0]);
    
    // Option 1: Sub Data
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(_quads[0])*_totalParticles, _quads);
    
    // Option 2: Data
    //  glBufferData(GL_ARRAY_BUFFER, sizeof(quads_[0]) * particleCount, quads_, GL_DYNAMIC_DRAW);
    
    // Option 3: Orphaning + glMapBuffer
    // glBufferData(GL_ARRAY_BUFFER, sizeof(_quads[0])*_totalParticles, nullptr, GL_STREAM_DRAW);
    // void *buf = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    // memcpy(buf, _quads, sizeof(_quads[0])*_totalParticles);
    // glUnmapBuffer(GL_ARRAY_BUFFER);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    CHECK_GL_ERROR_DEBUG();
}

// overriding draw method
void ParticleSystemQuadSIMD::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
    //CCASSERT( _particleIdx == 0 || _particleIdx == _particleCount, "Abnormal error in particle quad");
    //quad command
    
    if(_particleCount > 0)
    {
        _quadCommand.init(_globalZOrder, _texture->getName(), getGLProgramState(), _blendFunc, _quads, _particleCount, transform, flags);
        renderer->addCommand(&_quadCommand);
    }
}

void ParticleSystemQuadSIMD::setTotalParticles(int tp)
{
    // If we are setting the total number of particles to a number higher
    // than what is allocated, we need to allocate new arrays
    if( tp > _allocatedParticles )
    {
        // Allocate new memory
        size_t particlesSize = tp * sizeof(tParticle);
        size_t quadsSize = sizeof(_quads[0]) * tp * 1;
        size_t indicesSize = sizeof(_indices[0]) * tp * 6 * 1;
        
        mParticleData.release();
        if( !mParticleData.init(tp) )
        {
            CCLOG("Particle system: out of memory");
            return;
        }
        
        V3F_C4B_T2F_Quad* quadsNew = (V3F_C4B_T2F_Quad*)realloc(_quads, quadsSize);
        GLushort* indicesNew = (GLushort*)realloc(_indices, indicesSize);
        
        if ( quadsNew && indicesNew)
        {
            // Assign pointers
           
            _quads = quadsNew;
            _indices = indicesNew;
            
            // Clear the memory
            memset(_quads, 0, quadsSize);
            memset(_indices, 0, indicesSize);
            
            _allocatedParticles = tp;
        }
        else
        {
            // Out of memory, failed to resize some array
            if (quadsNew) _quads = quadsNew;
            if (indicesNew) _indices = indicesNew;
            
            CCLOG("Particle system: out of memory");
            return;
        }
        
        _totalParticles = tp;
        
        // Init particles
        if (_batchNode)
        {
            for (int i = 0; i < _totalParticles; i++)
            {
                mParticleData.atlasIndex[i] = i;
            }
        }
        
        initIndices();
        if (Configuration::getInstance()->supportsShareableVAO())
        {
            setupVBOandVAO();
        }
        else
        {
            setupVBO();
        }
        
        // fixed http://www.cocos2d-x.org/issues/3990
        // Updates texture coords.
        updateTexCoords();
    }
    else
    {
        _totalParticles = tp;
    }
    
    // fixed issue #5762
    // reset the emission rate
    setEmissionRate(_totalParticles / _life);
    
    resetSystem();
}

void ParticleSystemQuadSIMD::setupVBOandVAO()
{
    // clean VAO
    glDeleteBuffers(2, &_buffersVBO[0]);
    glDeleteVertexArrays(1, &_VAOname);
    GL::bindVAO(0);
    
    glGenVertexArrays(1, &_VAOname);
    GL::bindVAO(_VAOname);
    
#define kQuadSize sizeof(_quads[0].bl)
    
    glGenBuffers(2, &_buffersVBO[0]);
    
    glBindBuffer(GL_ARRAY_BUFFER, _buffersVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_quads[0]) * _totalParticles, _quads, GL_DYNAMIC_DRAW);
    
    // vertices
    glEnableVertexAttribArray(GLProgram::VERTEX_ATTRIB_POSITION);
    glVertexAttribPointer(GLProgram::VERTEX_ATTRIB_POSITION, 2, GL_FLOAT, GL_FALSE, kQuadSize, (GLvoid*) offsetof( V3F_C4B_T2F, vertices));
    
    // colors
    glEnableVertexAttribArray(GLProgram::VERTEX_ATTRIB_COLOR);
    glVertexAttribPointer(GLProgram::VERTEX_ATTRIB_COLOR, 4, GL_UNSIGNED_BYTE, GL_TRUE, kQuadSize, (GLvoid*) offsetof( V3F_C4B_T2F, colors));
    
    // tex coords
    glEnableVertexAttribArray(GLProgram::VERTEX_ATTRIB_TEX_COORD);
    glVertexAttribPointer(GLProgram::VERTEX_ATTRIB_TEX_COORD, 2, GL_FLOAT, GL_FALSE, kQuadSize, (GLvoid*) offsetof( V3F_C4B_T2F, texCoords));
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffersVBO[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(_indices[0]) * _totalParticles * 6, _indices, GL_STATIC_DRAW);
    
    // Must unbind the VAO before changing the element buffer.
    GL::bindVAO(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    CHECK_GL_ERROR_DEBUG();
}

void ParticleSystemQuadSIMD::setupVBO()
{
    glDeleteBuffers(2, &_buffersVBO[0]);
    
    glGenBuffers(2, &_buffersVBO[0]);
    
    glBindBuffer(GL_ARRAY_BUFFER, _buffersVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_quads[0]) * _totalParticles, _quads, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffersVBO[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(_indices[0]) * _totalParticles * 6, _indices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    
    CHECK_GL_ERROR_DEBUG();
}

void ParticleSystemQuadSIMD::listenRendererRecreated(EventCustom* event)
{
    //when comes to foreground in android, _buffersVBO and _VAOname is a wild handle
    //before recreating, we need to reset them to 0
    memset(_buffersVBO, 0, sizeof(_buffersVBO));
    if (Configuration::getInstance()->supportsShareableVAO())
    {
        _VAOname = 0;
        setupVBOandVAO();
    }
    else
    {
        setupVBO();
    }
}

bool ParticleSystemQuadSIMD::allocMemory()
{
    CCASSERT( !_batchNode, "Memory should not be alloced when not using batchNode");
    
    CC_SAFE_FREE(_quads);
    CC_SAFE_FREE(_indices);
    
    _quads = (V3F_C4B_T2F_Quad*)malloc(_totalParticles * sizeof(V3F_C4B_T2F_Quad));
    _indices = (GLushort*)malloc(_totalParticles * 6 * sizeof(GLushort));
    
    if( !_quads || !_indices)
    {
        CCLOG("cocos2d: Particle system: not enough memory");
        CC_SAFE_FREE(_quads);
        CC_SAFE_FREE(_indices);
        
        return false;
    }
    
    memset(_quads, 0, _totalParticles * sizeof(V3F_C4B_T2F_Quad));
    memset(_indices, 0, _totalParticles * 6 * sizeof(GLushort));
    
    return true;
}

void ParticleSystemQuadSIMD::setBatchNode(ParticleBatchNode * batchNode)
{
    if( _batchNode != batchNode )
    {
        ParticleBatchNode* oldBatch = _batchNode;
        
            
        _batchNode = batchNode; // weak reference
        
        if( batchNode )
        {
            //each particle needs a unique index
            for (int i = 0; i < _totalParticles; i++)
            {
                mParticleData.atlasIndex[i] = i;
            }
        }
        //ParticleSystem::setBatchNode(batchNode);
        
        // NEW: is self render ?
        if( ! batchNode )
        {
            allocMemory();
            initIndices();
            setTexture(oldBatch->getTexture());
            if (Configuration::getInstance()->supportsShareableVAO())
            {
                setupVBOandVAO();
            }
            else
            {
                setupVBO();
            }
        }
        // OLD: was it self render ? cleanup
        else if( !oldBatch )
        {
            // copy current state to batch
            V3F_C4B_T2F_Quad *batchQuads = _batchNode->getTextureAtlas()->getQuads();
            V3F_C4B_T2F_Quad *quad = &(batchQuads[_atlasIndex] );
            memcpy( quad, _quads, _totalParticles * sizeof(_quads[0]) );
            
            CC_SAFE_FREE(_quads);
            CC_SAFE_FREE(_indices);
            
            glDeleteBuffers(2, &_buffersVBO[0]);
            memset(_buffersVBO, 0, sizeof(_buffersVBO));
            if (Configuration::getInstance()->supportsShareableVAO())
            {
                glDeleteVertexArrays(1, &_VAOname);
                GL::bindVAO(0);
                _VAOname = 0;
            }
        }
    }
}

ParticleSystemQuadSIMD * ParticleSystemQuadSIMD::create() {
    ParticleSystemQuadSIMD *particleSystemQuad = new (std::nothrow) ParticleSystemQuadSIMD();
    if (particleSystemQuad && particleSystemQuad->init())
    {
        particleSystemQuad->autorelease();
        return particleSystemQuad;
    }
    CC_SAFE_DELETE(particleSystemQuad);
    return nullptr;
}

std::string ParticleSystemQuadSIMD::getDescription() const
{
    return StringUtils::format("<ParticleSystemQuadSIMD | Tag = %d, Total Particles = %d>", _tag, _totalParticles);
}

NS_CC_END


