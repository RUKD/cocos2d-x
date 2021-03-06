//
//  CCParticleSystemQuadNeon.h
//  cocos2d_libs
//
//  Created by RUKD on 15/3/30.
//
//

#ifndef __cocos2d_libs__CCParticleSystemQuadSOA__
#define __cocos2d_libs__CCParticleSystemQuadSOA__

#include "2d/CCParticleSystemQuad.h"
#include "2d/CCParticleSystemQuadSIMD.h"

NS_CC_BEGIN

class SpriteFrame;
class EventCustom;


/**
 * @addtogroup particle_nodes
 * @{
 */

/** @brief ParticleSystemQuad is a subclass of ParticleSystem
 
 It includes all the features of ParticleSystem.
 
 Special features and Limitations:
 - Particle size can be any float number.
 - The system can be scaled
 - The particles can be rotated
 - It supports subrects
 - It supports batched rendering since 1.1
 @since v0.8
 */
class CC_DLL ParticleSystemQuadSOA : public ParticleSystemQuad
{
public:
    
    /** creates a Particle Emitter */
    static ParticleSystemQuadSOA * create();
    /** creates a Particle Emitter with a number of particles */
    static ParticleSystemQuadSOA * createWithTotalParticles(int numberOfParticles);
    /** creates an initializes a ParticleSystemQuad from a plist file.
     This plist files can be created manually or with Particle Designer:
     */
    static ParticleSystemQuadSOA * create(const std::string& filename);
    /** creates a Particle Emitter with a dictionary */
    static ParticleSystemQuadSOA * create(ValueMap &dictionary);
    
    /** Sets a new SpriteFrame as particle.
     WARNING: this method is experimental. Use setTextureWithRect instead.
     @since v0.99.4
     */
    void setDisplayFrame(SpriteFrame *spriteFrame);
    
    /** Sets a new texture with a rect. The rect is in Points.
     @since v0.99.4
     * @js NA
     * @lua NA
     */
    void setTextureWithRect(Texture2D *texture, const Rect& rect);
    
    /** listen the event that renderer was recreated on Android/WP8
     * @js NA
     * @lua NA
     */
    void listenRendererRecreated(EventCustom* event);
    
    /**
     * @js NA
     * @lua NA
     */
    virtual void setTexture(Texture2D* texture) override;
    /**
     * @js NA
     * @lua NA
     */
    virtual void updateQuadWithParticle(tParticle* particle, const Vec2& newPosition) override;
    /**
     * @js NA
     * @lua NA
     */
    virtual void postStep() override;
    /**
     * @js NA
     * @lua NA
     */
    virtual void draw(Renderer *renderer, const Mat4 &transform, uint32_t flags) override;
    
    /**
     * @js NA
     * @lua NA
     */
    virtual void setBatchNode(ParticleBatchNode* batchNode) override;
    /**
     * @js NA
     * @lua NA
     */
    virtual void setTotalParticles(int tp) override;
    
    virtual std::string getDescription() const override;
public:
    //neon overide
    virtual bool addParticle();
    virtual bool addParticles(int count);
    virtual bool initParticleByIndex(int index);
    virtual void resetSystem();
    
    virtual void update(float dt) override;
    void updateVertexData();
    
    
CC_CONSTRUCTOR_ACCESS:
    /**
     * @js ctor
     */
    ParticleSystemQuadSOA();
    /**
     * @js NA
     * @lua NA
     */
    virtual ~ParticleSystemQuadSOA();
    
    // Overrides
    /**
     * @js NA
     * @lua NA
     */
    virtual bool initWithTotalParticles(int numberOfParticles) override;
    
protected:

    bool allocMemory();
    
    ParticleDataSOA    mParticleData;
    
private:
    CC_DISALLOW_COPY_AND_ASSIGN(ParticleSystemQuadSOA);
};

// end of particle_nodes group
/// @}

NS_CC_END

#endif /* defined(__cocos2d_libs__CCParticleSystemQuadSOA__) */
