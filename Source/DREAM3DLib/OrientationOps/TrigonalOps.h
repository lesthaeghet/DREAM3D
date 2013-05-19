/* ============================================================================
 * Copyright (c) 2010, Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2010, Dr. Michael A. Groeber (US Air Force Research Laboratories
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Michael A. Groeber, Michael A. Jackson, the US Air Force,
 * BlueQuartz Software nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  This code was written under United States Air Force Contract number
 *                           FA8650-07-D-5800
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef TrigonalOPS_H_
#define TrigonalOPS_H_

#include "MXA/Common/MXASetGetMacros.h"
#include "DREAM3DLib/Common/OrientationMath.h"


namespace TrigonalMath {
  namespace Detail {
    static const float TrigQuatSym[6][5] = {{0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f, 1.000000000f},
                         {0.000000000f, 0.000000000f, 0.000000000f, 0.866025400f, 0.500000000f},
                         {0.000000000f, 0.000000000f, 0.000000000f, 0.866025400f, -0.50000000f},
 						 {0.000000000f, 0.86602540f, 0.500000000f, 0.000000000f, 0.000000000f},
						 {0.000000000f, 0.000000000f, 1.000000000f, 0.000000000f, 0.000000000f},
                         {0.000000000f, -0.866025400f, 0.500000000f, 0.000000000f, 0.000000000}};
  }
}
/**
 * @class TrigonalOps TrigonalOps.h DREAM3DLib/Common/OrientationOps/TrigonalOps.h
 * @brief
 * @author Michael A. Jackson for BlueQuartz Software
 * @author Michael A. Groeber for USAF Research Laboratory, Materials Lab
 * @date May 5, 2011
 * @version 1.0
 */
class DREAM3DLib_EXPORT TrigonalOps : public OrientationMath
{
  public:
    MXA_SHARED_POINTERS(TrigonalOps)
    MXA_TYPE_MACRO(TrigonalOps)
    MXA_STATIC_NEW_MACRO(TrigonalOps)

    TrigonalOps();
    virtual ~TrigonalOps();

    virtual int getODFSize() { return 31104; }
    virtual int getMDFSize() { return 31104; }
    virtual int getNumSymOps() { return 6; }

    virtual float getMisoQuat( float q1[5],float q2[5],float &n1min,float &n2min,float &n3min);
    virtual void getQuatSymOp(int i, float *q);
    virtual void getRodSymOp(int i, float *r);
    virtual void getODFFZRod(float &r1,float &r2, float &r3);
    virtual void getMDFFZRod(float &r1,float &r2, float &r3);
    virtual void getNearestQuat(float *q1, float *q2);
    virtual void getFZQuat(float *qr);
    virtual int getMisoBin(float r1, float r2, float r3);
    virtual void determineEulerAngles(int choose, float &synea1, float &synea2, float &synea3);
    virtual void determineRodriguesVector(int choose, float &r1, float &r2, float &r3);
    virtual int getOdfBin(float r1, float r2, float r3);
    virtual void getSchmidFactorAndSS(float loadx, float loady, float loadz, float &schmidfactor, int &slipsys);
    virtual void getmPrime(float q1[5], float q2[5], float LD[3], float &mPrime);
    virtual void getF1(float q1[5], float q2[5], float LD[3], bool maxSF, float &F1);
    virtual void getF1spt(float q1[5], float q2[5], float LD[3], bool maxSF, float &F1spt);
    virtual void getF7(float q1[5], float q2[5], float LD[3], bool maxSF, float &F7);

  protected:
    float _calcMisoQuat(const float quatsym[24][5], int numsym,
                  float q1[5], float q2[5],
                  float &n1, float &n2, float &n3);

  private:
    TrigonalOps(const TrigonalOps&); // Copy Constructor Not Implemented
    void operator=(const TrigonalOps&); // Operator '=' Not Implemented
};

#endif /* TrigonalOPS_H_ */