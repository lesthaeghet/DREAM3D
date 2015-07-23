/* ============================================================================
* Copyright (c) 2015 Iowa State University
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
* Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
* contributors may be used to endorse or promote products derived from this software
* without specific prior written permission.
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
* The code contained herein was funded by the followig contract:
*    United States Air Force Prime Contract FA8650-14-D-5224 Task Order #1
*
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ============================================================================
* SurfaceMeshToSolidModelIges uses code adapated from DREAM3D
* The DREAM3D licesne is reproduced below.
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ============================================================================
* Copyright (c) 2009-2015 BlueQuartz Software, LLC
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
* Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
* contributors may be used to endorse or promote products derived from this software
* without specific prior written permission.
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
* The code contained herein was partially funded by the followig contracts:
*    United States Air Force Prime Contract FA8650-07-D-5800
*    United States Air Force Prime Contract FA8650-10-D-5210
*    United States Prime Contract Navy N00173-07-C-2068
*
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef _SurfaceMeshToSolidModelIges_H_
#define _SurfaceMeshToSolidModelIges_H_

#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/Common/AbstractFilter.h"
#include "DREAM3DLib/Common/DREAM3DSetGetMacros.h"
#include <exception>

#define qh_dllimport 1

extern "C" {
#include <libqhull/libqhull.h>
#include <libqhull/qset.h>
#include <libqhull/geom.h>
#include <libqhull/poly.h>
#include <libqhull/io.h>
}

/**
 * @brief The SurfaceMeshToSolidModelIges class. See [Filter documentation](@ref surfacemeshtosolidmodeliges) for details.
 */
class SurfaceMeshToSolidModelIges : public AbstractFilter
{
    Q_OBJECT /* Need this for Qt's signals and slots mechanism to work */
  public:
	DREAM3D_SHARED_POINTERS(SurfaceMeshToSolidModelIges)
    DREAM3D_STATIC_NEW_MACRO(SurfaceMeshToSolidModelIges)
	DREAM3D_TYPE_MACRO_SUPER(SurfaceMeshToSolidModelIges, AbstractFilter)

	virtual ~SurfaceMeshToSolidModelIges();

    /* Place your input parameters here. You can use some of the DREAM3D Macros if you want to */
    DREAM3D_FILTER_PARAMETER(QString, OutputIgesDirectory)
    Q_PROPERTY(QString OutputIgesDirectory READ getOutputIgesDirectory WRITE setOutputIgesDirectory)

    DREAM3D_FILTER_PARAMETER(QString, OutputIgesPrefix)
    Q_PROPERTY(QString OutputIgesPrefix READ getOutputIgesPrefix WRITE setOutputIgesPrefix)

    DREAM3D_FILTER_PARAMETER(bool, GroupByPhase)
    Q_PROPERTY(bool GroupByPhase READ getGroupByPhase WRITE setGroupByPhase)

    DREAM3D_FILTER_PARAMETER(DataArrayPath, SurfaceMeshFaceLabelsArrayPath)
    Q_PROPERTY(DataArrayPath SurfaceMeshFaceLabelsArrayPath READ getSurfaceMeshFaceLabelsArrayPath WRITE setSurfaceMeshFaceLabelsArrayPath)

    DREAM3D_FILTER_PARAMETER(DataArrayPath, SurfaceMeshFacePhasesArrayPath)
    Q_PROPERTY(DataArrayPath SurfaceMeshFacePhasesArrayPath READ getSurfaceMeshFacePhasesArrayPath WRITE setSurfaceMeshFacePhasesArrayPath)

	DREAM3D_FILTER_PARAMETER(DataArrayPath, SurfaceMeshFeatureFaceLabelsArrayPath)
	Q_PROPERTY(DataArrayPath SurfaceMeshFeatureFaceLabelsArrayPath READ getSurfaceMeshFeatureFaceLabelsArrayPath WRITE setSurfaceMeshFeatureFaceLabelsArrayPath)

	DREAM3D_FILTER_PARAMETER(DataArrayPath, SurfaceMeshFeatureFaceIdsArrayPath)
	Q_PROPERTY(DataArrayPath SurfaceMeshFeatureFaceIdsArrayPath READ getSurfaceMeshFeatureFaceIdsArrayPath WRITE setSurfaceMeshFeatureFaceIdsArrayPath)

	//DREAM3D_FILTER_PARAMETER(QString, SurfaceDataContainerName)
	//Q_PROPERTY(QString SurfaceDataContainerName READ getSurfaceDataContainerName WRITE setSurfaceDataContainerName)

	DREAM3D_FILTER_PARAMETER(DataArrayPath, SurfaceMeshTriangleNormalsArrayPath)
	Q_PROPERTY(DataArrayPath SurfaceMeshTriangleNormalsArrayPath READ getSurfaceMeshTriangleNormalsArrayPath WRITE setSurfaceMeshTriangleNormalsArrayPath)

    /**
     * @brief getCompiledLibraryName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getCompiledLibraryName();

    /**
     * @brief newFilterInstance Reimplemented from @see AbstractFilter class
     */
    virtual AbstractFilter::Pointer newFilterInstance(bool copyFilterParameters);

    /**
     * @brief getGroupName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getGroupName();

    /**
     * @brief getSubGroupName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getSubGroupName();

    /**
     * @brief getHumanLabel Reimplemented from @see AbstractFilter class
     */
    virtual const QString getHumanLabel();

    /**
     * @brief setupFilterParameters Reimplemented from @see AbstractFilter class
     */
    virtual void setupFilterParameters();

    /**
     * @brief writeFilterParameters Reimplemented from @see AbstractFilter class
     */
    virtual int writeFilterParameters(AbstractFilterParametersWriter* writer, int index);

    /**
     * @brief readFilterParameters Reimplemented from @see AbstractFilter class
     */
    virtual void readFilterParameters(AbstractFilterParametersReader* reader, int index);

   /**
    * @brief execute Reimplemented from @see AbstractFilter class
    */
    virtual void execute();

    /**
    * @brief preflight Reimplemented from @see AbstractFilter class
    */
    virtual void preflight();

  signals:
    /**
     * @brief updateFilterParameters Emitted when the Filter requests all the latest Filter parameters
     * be pushed from a user-facing control (such as a widget)
     * @param filter Filter instance pointer
     */
    void updateFilterParameters(AbstractFilter* filter);

    /**
     * @brief parametersChanged Emitted when any Filter parameter is changed internally
     */
    void parametersChanged();

    /**
     * @brief preflightAboutToExecute Emitted just before calling dataCheck()
     */
    void preflightAboutToExecute();

    /**
     * @brief preflightExecuted Emitted just after calling dataCheck()
     */
    void preflightExecuted();

  protected:
	SurfaceMeshToSolidModelIges();

    /**
     * @brief dataCheck Checks for the appropriate parameter values and availability of arrays
     */
    void dataCheck();

  private:
    DEFINE_DATAARRAY_VARIABLE(int32_t, SurfaceMeshFaceLabels)
    DEFINE_DATAARRAY_VARIABLE(int32_t, SurfaceMeshFacePhases)
	DEFINE_DATAARRAY_VARIABLE(int32_t, SurfaceMeshFeatureFaceLabels)
	DEFINE_DATAARRAY_VARIABLE(int32_t, SurfaceMeshFeatureFaceIds)
	DEFINE_DATAARRAY_VARIABLE(double, SurfaceMeshTriangleNormals)
	DEFINE_DATAARRAY_VARIABLE(double, TestNormals)

    /**
     * @brief writeHeader Writes the header of the STL file
     * @param f File instance pointer
     * @param header Header to write to file
     * @param triCount Number of triangles
     * @return Integer error value
     */
    int32_t writeHeader(FILE* f, const QString& header, int32_t triCount);

    /**
     * @brief writeNumTrianglesToFile Writes the number of triangles to the STL file
     * @param filename Name of the output file
     * @param triCount Number of triangles
     * @return Integer error value
     */
    int32_t writeNumTrianglesToFile(const QString& filename, int32_t triCount);

	/** 
	 * @brief RecurseOrderedEdges Recursively builds lists of unique edge loops
	 * @param edgelist an unordered list of edge pairs
     * @param orderededges QList of a QList of a QList of int64_t's to contain the ordered edge pairs
	 * @param visitededges boolean QVector to track edge use or to mark bad points to be ignored
	 * @param curborder the state of the current border
	 * @param candidates the possible next candidates
	 * @return Non-zero on error or zero on success
	 */
	int32_t RecurseOrderedEdges(QList<QList<int64_t>> edgelist, QList<QList<QList<int64_t>>> orderededges, QVector<bool> &visitededges, QList<QList<int64_t>> &curborder, QList<int64_t> candidates);

	/**
	 * @brief RecurseTrianglesOnSurface Finds all adjacent triangles on the same surface and grain
	 * @param 
	 * @param checkedtriangles is a boolean QVector listing the triangles that have been checked
	 * @param t is an index to the starting triangle
	 * @param grain is an index to the grain of interest
	 * @param featurefacelabel is an index to the feature face label describing the surface
	 * @return QList of indexes for the triangles on the surface
	 */
	QList<int32_t> RecurseTrianglesOnSurface(ElementDynamicList::Pointer m_TriangleNeighbors, QVector<bool> &checkedtriangles, int64_t t, int32_t grain, int32_t featurefacelabel);


	/**
	 * @brief RecurseExternalTrianglesOnSurface Finds all adjacent triangles on the same surface and grain
	 * @param 
	 * @param checkedtriangles is a boolean QVector listing the triangles that have been checked
	 * @param t is an index to the starting triangle
	 * @param grain is an index to the grain of interest
	 * @param featurefacelabel is an index to the feature face label describing the surface
	 * @param n0, n1, n2 are the normal for the current plane
	 * @return QList of indexes for the triangles on the surface
	 */
	QList<int32_t> RecurseExternalTrianglesOnSurface(ElementDynamicList::Pointer m_TriangleNeighbors, QVector<bool> &checkedtriangles, int64_t t, int32_t grain, int32_t featurefacelabel, float n0, float n1, float n2);

	int64_t SurfMeshParams(const int64_t n, const int64_t m, const double *Q, double *uk, double *vl);
	double Distance3D(const int64_t xa, const int64_t ya, const int64_t xb, const int64_t yb, const int64_t n, const int64_t m, const double *Q);
	double Distance3D(const int64_t a, const int64_t b, const double *Q);
	void GlobalCurveInterp(int64_t n, double *Q, int64_t r, int64_t p, int64_t &m, double *U, double *P);
	int64_t FindSpan(int64_t n, int64_t p, double u, double *U);
	void BasisFunction(int64_t i, double u, int64_t p, double *U, double *N);
	void bandec(double **a, int64_t n, int64_t m1, int64_t m2, double **al, int64_t indx[], double *d);
	void banbks(double **a, int64_t n, int64_t m1, int64_t m2, double **al, int64_t indx[], double b[]);

	SurfaceMeshToSolidModelIges(const SurfaceMeshToSolidModelIges&); // Copy Constructor Not Implemented
	void operator=(const SurfaceMeshToSolidModelIges&); // Operator '=' Not Implemented
};

#endif /* _SurfaceMeshToSolidModelIges_H_ */
