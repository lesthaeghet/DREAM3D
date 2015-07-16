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


#include "SurfaceMeshToSolidModelIges.h"

#include <QtCore/QDir>

#ifdef DREAM3D_USE_PARALLEL_ALGORITHMS
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/partitioner.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/atomic.h>
#endif

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/FileSystemFilterParameter.h"
#include "DREAM3DLib/FilterParameters/LinkedBooleanFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"
#include "DREAM3DLib/DataArrays/DynamicListArray.hpp"
#include "DREAM3DLib/Math/GeometryMath.h"
#include "DREAM3DLib/Math/MatrixMath.h"
#include "DREAM3DLib/Math/DREAM3DMath.h"

#include "IO/IOConstants.h"
#include <stdexcept>

// Code to handle rectangle edge maping given two vectors describing that edge
class VectorEdgeIterator
{
	std::vector<float> m_xvals;
	std::vector<float> m_yvals;

public:
	VectorEdgeIterator(std::vector<float> xvals, std::vector<float>yvals) :	m_xvals(xvals),	m_yvals(yvals)	{}
	virtual ~VectorEdgeIterator() {}
	int64_t size() 	{ return (2 * (m_xvals.size() + m_yvals.size()) - 4);	}

	float distance(int64_t idx, float xval, float yval) {
		int64_t subs[2];
		ind2sub(idx, subs);

		float xd = m_xvals[subs[0]] - xval;
		float yd = m_yvals[subs[1]] - yval;

		return sqrt(xd*xd + yd*yd);
	}

	int64_t ind2gridind(int64_t firstidx, int64_t secondidx, int64_t idx){
		int64_t nextidx;

		if (idx < 0 || idx >= size())
			throw std::out_of_range("Edge index value out of range");

		// Are we decreasing or increasing
		if (firstidx > secondidx) // decreasing (CW)
		{
			nextidx = firstidx - idx;
			if (nextidx < 0)
			{
				nextidx = size() + nextidx;
			}
		}
		else // increasing (CCW)
		{
			nextidx = firstidx + idx;
			if (nextidx >= size())
			{
				nextidx = nextidx - size();
			}
		}

		return nextidx;
	}

	int64_t sub2ind(int64_t *sub)
	{
		int64_t retval;

		int64_t x = sub[0];
		int64_t y = sub[1];

		int64_t xsize = m_xvals.size();
		int64_t ysize = m_yvals.size();

		if (!(x == 0 || y == 0 || x == xsize - 1 || y == ysize - 1))
			throw std::out_of_range("Edge index values only");

		if (x >= 0 && x < xsize && y == 0)
		{
			retval = x;
		}
		else if (x == xsize - 1 && y > 0 && y < ysize - 1)
		{
			retval = xsize - 1 + y;
		}
		else if (x >= 0 && x < xsize && y == ysize - 1)
		{
			retval = (xsize - x - 1) + (xsize + ysize - 2);
		}
		else if (x == 0 && y > 0 && y < ysize - 1)
		{
			retval = (ysize - y - 1) + (2 * xsize + ysize - 3);
		}

		return retval;

	}

	void ind2sub(int64_t idx, int64_t *retval){
		
		int64_t xsize = m_xvals.size();
		int64_t ysize = m_yvals.size();
		
		if (idx < 0 || idx >= size())
			throw std::out_of_range("Edge index value out of range");

		if (idx >= 0 && idx < xsize)
		{
			retval[0] = idx;
			retval[1] = 0;
		}
		else if (idx >= xsize && idx < (xsize + ysize - 1))
		{
			retval[0] = xsize - 1;
			retval[1] = idx - xsize + 1;
		}
		else if (idx >= (xsize + ysize - 1) && idx < (2 * xsize + ysize - 2))
		{
			retval[0] = (xsize - 2) - (idx - (xsize + ysize - 1));
			retval[1] = ysize - 1;
		}
		else
		{
			retval[0] = 0;
			retval[1] = (ysize - 2) - (idx - (2 * xsize + ysize - 2));
		}
		
	}

};

class IndexConverter
{
	const int64_t m_xcnt;
	const int64_t m_ycnt;
	const int64_t m_zcnt;
	const int64_t m_xcntbyycnt;
	const int64_t m_ycntbyzcnt;
	const int64_t m_maxn;

public:
	IndexConverter(const int64_t xcnt, const int64_t ycnt, const int64_t zcnt) :
		m_xcnt(xcnt),
		m_ycnt(ycnt),
		m_zcnt(zcnt),
		m_xcntbyycnt(xcnt*ycnt),
		m_ycntbyzcnt(ycnt*zcnt),
		m_maxn(IndexConverter::max(xcnt, ycnt, zcnt) / 2) {}
	
	virtual ~IndexConverter() {}

	/**
	 * @brief Given x,y,z grid values, sub2ind returns the linear index for grid use
	 * @param x is the x index
	 * @param y is the y index
	 * @param z is the z index
	 * @return index value
	 */
	const int64_t sub2ind(const int64_t x, const int64_t y, const int64_t z) {
		return z + y * m_zcnt + x * m_ycnt * m_zcnt;
	}

	/**
	 * @brief Given an index value, return the x, y, z grid coordinates
	 * @param ind is the index with 0 <= ind < m_xsize*m_ysize*m_zsize
	 */
	void ind2sub(const int64_t ind, int64_t &x, int64_t &y, int64_t &z){
		x = ind / (m_ycntbyzcnt);
		y = (ind - (x * m_ycntbyzcnt)) / m_zcnt;
		z = ind - (x * m_ycntbyzcnt) - (y*m_zcnt);
	}

	/** 
	 * @brief Given an x,y index point in the grid and an iterator value,
	 * find the grid index for the idx point so that we are iterating radially
	 * from the start point for algorithms that don't really need to search the
	 * entire grid.
	 * @param x is the starting x value
	 * @param y is the starting y value
	 * @param curx is the current x value
	 * @param cury is the current y value
	 * @param idx is the current iterator value such that 0 <= x < m_xcnt*m_ycnt
	 * @param index suitable for grid access at that point
	 */
	const int64_t radit(const int64_t startx, const int64_t starty, int64_t &curx, int64_t &cury) {
		
		const int64_t n = IndexConverter::max(abs(startx - curx), abs(starty - cury));
		int64_t badcnt = 0;
		const int64_t breakcriteria = 4 * (2 * (n + 1) + 1) - 4;
		
		while (true)
		{
			if (curx == startx + n)
			{
				// right side
				if (cury == starty + n)
				{
					// we need to go left
					curx--;
				}
				else
				{
					// we need to go up
					cury++;
				}
			}
			else if (curx == startx - n)
			{
				// left side
				if (cury == starty - n)
				{
					// we need to go right
					curx++;
				}
				else
				{
					// we need to go down
					cury--;
				}
			}
			else if (cury == startx - n)
			{
				// bottom side
				// we need to go right
				curx++;
			}
			else if (cury == startx + n)
			{
				// top side
				// we need to go left
				cury--;
			}
			else
			{
				// something is wrong
				assert(false);
			}

			if (curx > 0 && curx < m_xcnt && cury > 0 && cury < m_ycnt)
				break;
			else
				badcnt++;

			if (badcnt > breakcriteria)
				return -1;

		}

		return sub2ind(curx, cury, 0);

	}

	const int64_t max(const int64_t x, const int64_t y, const int64_t z) const {
		if (x >= y && x >= z)
			return x;
		else if (y >= x && y >= z)
			return y;
		else if (z >= x && z >= y)
			return z;
		else
			return -1;
	}

	const int64_t max(const int64_t x, const int64_t y) const {
		if (x >= y)
			return x;
		else if (y >= x)
			return y;
		else
			return -1;
	}

};

// Code to handle threading for ray tracing algorithm
class RayTraceSurf
{
	int32_t m_xcnt;
	int32_t m_ycnt;
	std::vector<float> m_xvals;
	std::vector<float> m_yvals;
	float m_rotmat[3][3];
	float *m_ll; 
	float *m_ur; 
	TriangleGeom::Pointer m_triangleGeom;
	QList<QList<int32_t>>::iterator m_cursurf;
	float *m_griddedsurf;
	const double *m_SurfaceMeshTriangleNormals;
	SurfaceMeshToSolidModelIges* m_cls;


public:
	RayTraceSurf(int32_t xcnt, int32_t ycnt,
		std::vector<float> xvals, std::vector<float> yvals, float rotmat[3][3],
		float ll[3], float ur[3], TriangleGeom::Pointer triangleGeom,
		QList<QList<int32_t>>::iterator cursurf, float *griddedsurf,
		const double *trianglenorms, SurfaceMeshToSolidModelIges* cls) :
		m_xcnt(xcnt),
		m_ycnt(ycnt),
		m_xvals(xvals),
		m_yvals(yvals),
		m_ll(ll),
		m_ur(ur),
		m_triangleGeom(triangleGeom),
		m_cursurf(cursurf),
		m_griddedsurf(griddedsurf),
		m_SurfaceMeshTriangleNormals(trianglenorms),
		m_cls(cls)
	{
		m_rotmat[0][0] = rotmat[0][0];
		m_rotmat[0][1] = rotmat[0][1];
		m_rotmat[0][2] = rotmat[0][2];
		m_rotmat[1][0] = rotmat[1][0];
		m_rotmat[1][1] = rotmat[1][1];
		m_rotmat[1][2] = rotmat[1][2];
		m_rotmat[2][0] = rotmat[2][0];
		m_rotmat[2][1] = rotmat[2][1];
		m_rotmat[2][2] = rotmat[2][2];
	}
	virtual ~RayTraceSurf() {}

	void generate(int64_t start, int64_t end, int32_t xcnt, int32_t ycnt,
		std::vector<float> xvals, std::vector<float> yvals, const float rotmat[3][3],
		float ll[3], float ur[3], TriangleGeom::Pointer triangleGeom,
		QList<QList<int32_t>>::iterator cursurf, float *griddedsurf, 
		const double *m_SurfaceMeshTriangleNormals, SurfaceMeshToSolidModelIges* cls) const
	{

		float tmppt1[3];
		float tmpptr1[3];
		float tmppt2[3];
		float tmpptr2[3];
		tmppt1[2] = ll[2];
		tmppt2[2] = ur[2];


		// We can now start directly searching for grid values
		for (int32_t i = start; i < end; i++)
		{
			for (int32_t j = 0; j < ycnt; j++)
			{
				//QString ss = QObject::tr("Ray Tracing Grid Point %1/%2").arg(i*ycnt + j).arg((end-start)*ycnt);
				//cls->notifyStatusMessage(cls->getMessagePrefix(), cls->getHumanLabel(), ss);
				if (cls->getCancel() == true) { return; }

				tmppt1[0] = xvals[i];
				tmppt1[1] = yvals[j];
				tmppt2[0] = xvals[i];
				tmppt2[1] = yvals[j];
				MatrixMath::Multiply3x3with3x1(rotmat, tmppt1, tmpptr1);
				MatrixMath::Multiply3x3with3x1(rotmat, tmppt2, tmpptr2);

				float newgridpt[3];
				char code;

				// Loop all of the triangles looking for the intersecting triangle
				for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
				{

					float vert1[3], vert2[3], vert3[3];
					triangleGeom->getVertCoordsAtTri((*t), vert1, vert2, vert3);

					float tn[3] = { m_SurfaceMeshTriangleNormals[3 * (*t) + 0],
								    m_SurfaceMeshTriangleNormals[3 * (*t) + 1],
						            m_SurfaceMeshTriangleNormals[3 * (*t) + 2] };
					float rn[3] = { tmpptr2[0] - tmpptr1[0],
									tmpptr2[1] - tmpptr1[1],
									tmpptr2[2] - tmpptr1[2] };
					float norm = sqrt(rn[0] * rn[0] + rn[1] * rn[1] + rn[2] * rn[2]);
					rn[0] = rn[0] / norm;
					rn[1] = rn[1] / norm;
					rn[2] = rn[2] / norm;

					float angle = GeometryMath::AngleBetweenVectors(tn, rn);
					if (angle > 1.047 && angle < 2.094)
						code = '0';  // This will prevent ray tracing from matching values that are getting too close to parallel
					else
						code = GeometryMath::RayIntersectsTriangle(vert1, vert2, vert3, tmpptr1, tmpptr2, newgridpt);

					// We've got a match and a point
					if (code == 'f' || code == 'v' || code == 'e' || code == 'E' || code == 'V' || code == 'F')
						break;

				}

				// Make sure we actually had a match and set NAN if we didn't
				if (code != 'f' && code != 'v' && code != 'e' && code != 'E' && code != 'V' && code != 'F')
				{
					newgridpt[0] = nanf(""); newgridpt[1] = nanf(""); newgridpt[2] = nanf("");
				}

				// Save the grid point value
				griddedsurf[0 + j * 3 + i*ycnt * 3] = newgridpt[0];
				griddedsurf[1 + j * 3 + i*ycnt * 3] = newgridpt[1];
				griddedsurf[2 + j * 3 + i*ycnt * 3] = newgridpt[2];

			}
		}


	}

#ifdef DREAM3D_USE_PARALLEL_ALGORITHMS
	void operator()(const tbb::blocked_range<int64_t>& r) const
	{
		generate(r.begin(), r.end(), m_xcnt, m_ycnt, m_xvals, m_yvals, m_rotmat, m_ll, m_ur, m_triangleGeom, m_cursurf, m_griddedsurf, m_SurfaceMeshTriangleNormals, m_cls);
	}
#endif
};


// -----------------------------------------------------------------------------
// @
// Class Constructor
// 
// We need the following variables:
// m_SurfaceMeshFaceLabels, m_SurfaceMeshFacePhases,
// m_SurfaceMeshFeatureFaceLabels, m_SurfaceMeshFeatureFaceIds,
// m_TrianglesContainingVert, m_SurfaceMeshTriangleNormals
// and all paths as needed
// -----------------------------------------------------------------------------

class IntersectingBoundariesException : public std::exception
{
	virtual const char* what() const throw()
	{
		return "Intersecting independent boundaries have been found and are not presesntly supported by this filter.";
	}
} IntersectingBoundariesException;

class OpenBoundaryException : public std::exception
{
	virtual const char* what() const throw()
	{
		return "An open boundary has been found.";
	}
} OpenBoundaryException;

/**
SurfaceMeshToSolidModelIges Class Constructor
*/
SurfaceMeshToSolidModelIges::SurfaceMeshToSolidModelIges() :
  AbstractFilter(),
  m_OutputIgesDirectory(""),
  m_OutputIgesPrefix(""),
  m_GroupByPhase(false),
  m_SurfaceMeshFaceLabelsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::FaceAttributeMatrixName, DREAM3D::FaceData::SurfaceMeshFaceLabels),
  m_SurfaceMeshFacePhasesArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::FaceAttributeMatrixName, DREAM3D::FaceData::SurfaceMeshFacePhases),
  m_SurfaceMeshFeatureFaceLabelsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::FaceFeatureAttributeMatrixName, DREAM3D::FaceData::SurfaceMeshFaceLabels),
  m_SurfaceMeshFeatureFaceIdsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::FaceAttributeMatrixName, DREAM3D::FaceData::SurfaceMeshFeatureFaceId),
  //m_SurfaceDataContainerName(DREAM3D::Defaults::DataContainerName),
  m_SurfaceMeshTriangleNormalsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::FaceAttributeMatrixName, DREAM3D::FaceData::SurfaceMeshFaceNormals),
  m_SurfaceMeshFaceLabels(NULL),
  m_SurfaceMeshFacePhases(NULL),
  m_SurfaceMeshFeatureFaceLabels(NULL),
  m_SurfaceMeshFeatureFaceIds(NULL),
  m_SurfaceMeshTriangleNormals(NULL),
  m_TestNormals(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
SurfaceMeshToSolidModelIges::~SurfaceMeshToSolidModelIges()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(FileSystemFilterParameter::New("Output IGES Directory", "OutputIgesDirectory", FilterParameterWidgetType::OutputPathWidget, getOutputIgesDirectory(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("IGES File Prefix", "OutputIgesPrefix", FilterParameterWidgetType::StringWidget, getOutputIgesPrefix(), FilterParameter::Parameter));
  QStringList linkedProps("SurfaceMeshFacePhasesArrayPath");
  parameters.push_back(LinkedBooleanFilterParameter::New("Group Files By Ensemble", "GroupByPhase", getGroupByPhase(), linkedProps, FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Face Labels", "SurfaceMeshFaceLabelsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSurfaceMeshFaceLabelsArrayPath(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Face Phases", "SurfaceMeshFacePhasesArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSurfaceMeshFacePhasesArrayPath(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Feature Face Labels", "SurfaceMeshFeatureFaceLabelsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSurfaceMeshFeatureFaceLabelsArrayPath(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Feature Face Ids", "SurfaceMeshFeatureFaceIdsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSurfaceMeshFeatureFaceIdsArrayPath(), FilterParameter::RequiredArray, ""));
  //parameters.push_back(FilterParameter::New("Data Container", "SurfaceDataContainerName", FilterParameterWidgetType::DataContainerSelectionWidget, getSurfaceDataContainerName(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Face Normals", "SurfaceMeshTriangleNormalsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSurfaceMeshTriangleNormalsArrayPath(), FilterParameter::RequiredArray, ""));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setSurfaceMeshFacePhasesArrayPath(reader->readDataArrayPath("SurfaceMeshFacePhasesArrayPath", getSurfaceMeshFacePhasesArrayPath() ) );
  setSurfaceMeshFaceLabelsArrayPath(reader->readDataArrayPath("SurfaceMeshFaceLabelsArrayPath", getSurfaceMeshFaceLabelsArrayPath() ) );
  setOutputIgesDirectory( reader->readString( "OutputIgesDirectory", getOutputIgesDirectory() ) );
  setOutputIgesPrefix( reader->readString( "OutputIgesPrefix", getOutputIgesPrefix() ) );
  setSurfaceMeshFeatureFaceLabelsArrayPath(reader->readDataArrayPath("SurfaceMeshFeatureFaceLabelsArrayPath", getSurfaceMeshFeatureFaceLabelsArrayPath()));
  //setSurfaceDataContainerName(reader->readString("SurfaceDataContainerName", getSurfaceDataContainerName()));
  setSurfaceMeshTriangleNormalsArrayPath(reader->readDataArrayPath("SurfaceMeshTriangleNormalsArrayPath", getSurfaceMeshTriangleNormalsArrayPath()));
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int SurfaceMeshToSolidModelIges::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(SurfaceMeshFacePhasesArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(SurfaceMeshFaceLabelsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(SurfaceMeshFeatureFaceLabelsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(SurfaceMeshFeatureFaceIdsArrayPath)
  //DREAM3D_FILTER_WRITE_PARAMETER(SurfaceDataContainerName)
  DREAM3D_FILTER_WRITE_PARAMETER(SurfaceMeshTriangleNormalsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(OutputIgesDirectory)
  DREAM3D_FILTER_WRITE_PARAMETER(OutputIgesPrefix)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::dataCheck()
{
  setErrorCondition(0);

  TriangleGeom::Pointer triangles = getDataContainerArray()->getPrereqGeometryFromDataContainer<TriangleGeom, AbstractFilter>(this, getSurfaceMeshFaceLabelsArrayPath().getDataContainerName());
  
  QVector<IDataArray::Pointer> dataArrays;

  if(getErrorCondition() >= 0) { dataArrays.push_back(triangles->getTriangles()); }

  if (m_OutputIgesDirectory.isEmpty() == true)
  {
    setErrorCondition(-1003);
    notifyErrorMessage(getHumanLabel(), "The output directory must be set", -1003);
  }

  QVector<size_t> cDims(1, 2);

  // Surface Mesh Face Labels (numTriangles x 2)
  m_SurfaceMeshFaceLabelsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getSurfaceMeshFaceLabelsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_SurfaceMeshFaceLabelsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_SurfaceMeshFaceLabels = m_SurfaceMeshFaceLabelsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  if(getErrorCondition() >= 0) { dataArrays.push_back(m_SurfaceMeshFaceLabelsPtr.lock()); }

  // Surface Mesh Face Phases (numTriangles x 2)
  if (m_GroupByPhase == true)
  {
    m_SurfaceMeshFacePhasesPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getSurfaceMeshFacePhasesArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_SurfaceMeshFacePhasesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    { m_SurfaceMeshFacePhases = m_SurfaceMeshFacePhasesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
    if(getErrorCondition() >= 0) { dataArrays.push_back(m_SurfaceMeshFacePhasesPtr.lock()); }
  }

  // Surface Mesh Feature Face Labels (numUniqueInternalBoundaries x 2)
  m_SurfaceMeshFeatureFaceLabelsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getSurfaceMeshFeatureFaceLabelsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_SurfaceMeshFeatureFaceLabelsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {	  m_SurfaceMeshFeatureFaceLabels = m_SurfaceMeshFeatureFaceLabelsPtr.lock()->getPointer(0);  } /* Now assign the raw pointer to data from the DataArray<T> object */
  //if (getErrorCondition() >= 0) { dataArrays.push_back(m_SurfaceMeshFeatureFaceLabelsPtr.lock()); } // This should obviously fail

  // Surface Mesh Feature Face Ids (numTriangles x 1)
  cDims[0] = 1;
  m_SurfaceMeshFeatureFaceIdsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getSurfaceMeshFeatureFaceIdsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_SurfaceMeshFeatureFaceIdsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {	  m_SurfaceMeshFeatureFaceIds = m_SurfaceMeshFeatureFaceIdsPtr.lock()->getPointer(0);  } /* Now assign the raw pointer to data from the DataArray<T> object */
  if (getErrorCondition() >= 0) { dataArrays.push_back(m_SurfaceMeshFeatureFaceIdsPtr.lock()); }

  // Surface Mesh Triangle Normals (numTriangles x 3)
  cDims[0] = 3;
  m_SurfaceMeshTriangleNormalsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<double>, AbstractFilter>(this, getSurfaceMeshTriangleNormalsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_SurfaceMeshTriangleNormalsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {	  m_SurfaceMeshTriangleNormals = m_SurfaceMeshTriangleNormalsPtr.lock()->getPointer(0);  } /* Now assign the raw pointer to data from the DataArray<T> object */
  if (getErrorCondition() >= 0) { dataArrays.push_back(m_SurfaceMeshTriangleNormalsPtr.lock()); }

  DataArrayPath tmpPath;
  tmpPath.update(getSurfaceMeshTriangleNormalsArrayPath().getDataContainerName(), getSurfaceMeshTriangleNormalsArrayPath().getAttributeMatrixName(), "TmpNormals");
  m_TestNormalsPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<double>, AbstractFilter>(this, tmpPath, 0, cDims);
  if (NULL != m_TestNormalsPtr.lock().get())
  { m_TestNormals = m_TestNormalsPtr.lock()->getPointer(0); }
  if (getErrorCondition() >= 0) { dataArrays.push_back(m_TestNormalsPtr.lock()); }

  getDataContainerArray()->validateNumberOfTuples<AbstractFilter>(this, dataArrays);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::preflight()
{
  setInPreflight(true);
  emit preflightAboutToExecute();
  emit updateFilterParameters(this);
  dataCheck();
  emit preflightExecuted();
  setInPreflight(false);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::execute()
{
	int32_t err = 0;	// Return value storage for functions that return integer error value
	
	//  Run a data check and ensure we have everything we need to continue
	//  dataCheck will ensure that m_SurfaceMeshFaceLabels, m_SurfaceMeshFacePhases, (From Surface Mesh Generation - QuickSurfaceMesh, etc)
	//  m_SurfaceMeshFeatureFaceLabels, m_SurfaceMeshFeatureFaceIds, (From SharedFeatureFaceFilter)
	//  m_TrianglesContainingVert, (From GenerateGeometryConnectivity/GeometryHelpers)
	//  m_SurfaceMeshTriangleNormals (From TriangleNormalsFilter) exist, are not null, and are sized appropriately.
	setErrorCondition(0);
	dataCheck();
	if(getErrorCondition() < 0) { return; }

	#ifdef DREAM3D_USE_PARALLEL_ALGORITHMS
		tbb::task_scheduler_init init;
		bool doParallel = true;
	#endif

	// Make sure any directory path is also available as the user may have just typed
	// in a path without actually creating the full path
	QDir igesDir(getOutputIgesDirectory());		// Output Directory
	if (!igesDir.mkpath("."))
	{
		QString ss = QObject::tr("Error creating parent path '%1'").arg(getOutputIgesDirectory());
		notifyErrorMessage(getHumanLabel(), ss, -1);
		setErrorCondition(-1);
		return;
	}

	// Load in the geometry information
	TriangleGeom::Pointer triangleGeom = getDataContainerArray()->getDataContainer(getSurfaceMeshFaceLabelsArrayPath().getDataContainerName())->getGeometryAs<TriangleGeom>(); // Pointer to geometry
	float* nodes = triangleGeom->getVertexPointer(0); // Triangle vertex point locations
	int64_t* triangles = triangleGeom->getTriPointer(0); // Triangle vertex index values
	int64_t nTriangles = triangleGeom->getNumberOfTris(); // Number of triangles
	ElementDynamicList::Pointer m_TrianglesContainingVert = triangleGeom->getElementsContainingVert();  // List of triangles containing a vertex point
	ElementDynamicList::Pointer m_TriangleNeighbors = triangleGeom->getElementNeighbors(); // Mapping of triangle neighbors

	// Get Number of Unique Solid Features (Surface Mesh Face Labels and Surface Mesh Face Phases)
	QMap<int32_t, int32_t> uniqueGrainIdtoPhase; // Map of unique solid features
	if (m_GroupByPhase == true)
	{
		for (int64_t i = 0; i < nTriangles; i++)
		{
			uniqueGrainIdtoPhase.insert(m_SurfaceMeshFaceLabels[i * 2], m_SurfaceMeshFacePhases[i * 2]);
			uniqueGrainIdtoPhase.insert(m_SurfaceMeshFaceLabels[i * 2 + 1], m_SurfaceMeshFacePhases[i * 2 + 1]);
		}
	}
	else
	{
		for (int64_t i = 0; i < nTriangles; i++)
		{
			uniqueGrainIdtoPhase.insert(m_SurfaceMeshFaceLabels[i * 2], 0);
			uniqueGrainIdtoPhase.insert(m_SurfaceMeshFaceLabels[i * 2 + 1], 0);
		}
	}

	// Unique Grain ID
	int32_t grain = 0;

	// Count of Number of Elements in Surface Mesh Face Labels
	size_t uniqueFaceCount = m_SurfaceMeshFeatureFaceLabelsPtr.lock()->getNumberOfTuples();

	// Loop Over Each Grain
	for (QMap<int32_t, int32_t>::iterator spinIter = uniqueGrainIdtoPhase.begin(); spinIter != uniqueGrainIdtoPhase.end(); ++spinIter)
	{
		grain = spinIter.key();

		// Make sure we ignore grain -1
		if (grain == -1)
			continue;

		// We need to build the list of unique surfaces for this grain.
		// We can use the output from the shared feature face filter to speed this up.
		// However, we need to additionally go back through and look at the triangles
		// that share a surface with the outside and check them against the normals to
		// determine how many unique faces there are on the outer edges for this grain

		// Variable Declaration
		QList<int32_t> uniqueInternalSurfaces;			// Map to store a list of unique internal surfaces
		int32_t externalSurfaceFaceLabel = -1;			// Variable to store the label number of the one instance in the face labels list for the external surface
		int32_t uniqueExternalSurfacesCount = 0;		// Count of the number of unique external surfaces
		QList<double> uniqueExternalSurfaceNormals;		// List to store a list of unique external surfaces
		QList<QList<int32_t> > internalSurfaceTriangles;// List of internal surface triangles for each found surface
		QList<QList<int32_t> > externalSurfaceTriangles;// List of external surface triangles for each found surface

		// Let's first look for internal faces on this grain
		for (int32_t facecnt = 0; facecnt < uniqueFaceCount; facecnt++)
		{
			if (m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 0] == grain && m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 1] > 0)
				uniqueInternalSurfaces << facecnt;
			else if (m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 1] == grain && m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 0] > 0)
				uniqueInternalSurfaces << facecnt;
			else if (m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 0] == grain && m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 1] == -1)
				externalSurfaceFaceLabel = facecnt;
			else if (m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 1] == grain && m_SurfaceMeshFeatureFaceLabels[2 * facecnt + 0] == -1)
				externalSurfaceFaceLabel = facecnt;
		}

		// Now let's find all of the triangles with those feature faces and grain ids so we can iterate them later
		for (int32_t facecnt = 0; facecnt < uniqueInternalSurfaces.size(); ++facecnt)
		{
			// Prepare storage
			QVector<bool> checkedtriangles(nTriangles, false);
			int64_t surfcnt = 0;
			// Let's look for matching triangles
			for (int64_t t = 0; t < nTriangles; ++t)
			{
				// Have we found a triangle on this grain and surface
				if (checkedtriangles[t] == false && (m_SurfaceMeshFeatureFaceIds[t] == uniqueInternalSurfaces.at(facecnt) && (m_SurfaceMeshFaceLabels[2*t] == grain || m_SurfaceMeshFaceLabels[2*t+1] == grain)))
				{
					// Let's recurse until we find all of the adjoining triangles
					internalSurfaceTriangles << RecurseTrianglesOnSurface(m_TriangleNeighbors, checkedtriangles, t, grain, uniqueInternalSurfaces.at(facecnt));
					++surfcnt;										
				}
				else
				{
					checkedtriangles[t] = true;
				}
			}
			if (surfcnt > 1)
			{
				// We found more surfaces than we should have
				// Let's update the uniqueInternalSurfaces list to reflect this
				for (int64_t k = 1; k < surfcnt; ++k)
				{
					uniqueInternalSurfaces.insert(facecnt, uniqueInternalSurfaces[facecnt]);
					++facecnt;
				}
			}
		}

		// Now, let's deal with the more complicated problem of external faces on this grain
		// Fortunately, we can keep track of the triangle ids for each face while we're looping through

		// Make sure there are actually external faces before looping over all triangles
		if (externalSurfaceFaceLabel > -1)
		{
			// Let's find the face feature id that matches 
			for (int64_t t = 0; t < nTriangles; ++t)
			{
				// Have we found a triangle on the external surface
				if (m_SurfaceMeshFeatureFaceIds[t] == externalSurfaceFaceLabel && (m_SurfaceMeshFaceLabels[2*t] == grain || m_SurfaceMeshFaceLabels[2*t+1]))
				{
					// Let's see if we've found a triangle with the same normal as a previous one
					bool found = false;
					for (int32_t i = 0; i < uniqueExternalSurfacesCount; i++)
					{
						// This is horribly inefficient - should find a better method
						if (m_SurfaceMeshTriangleNormals[3 * t] == uniqueExternalSurfaceNormals[3 * i] &&
							m_SurfaceMeshTriangleNormals[3 * t + 1] == uniqueExternalSurfaceNormals[3 * i + 1] &&
							m_SurfaceMeshTriangleNormals[3 * t + 2] == uniqueExternalSurfaceNormals[3 * i + 2])
						{
							// We found it - store the triangle id
							externalSurfaceTriangles[i] << t;
							// Break out of the loop
							found = true;
							break;
						}
					}

					// We didn't find a tirangle with the same normal - so let's add this to our list
					if (!found)
					{
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t];
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t + 1];
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t + 2];
						externalSurfaceTriangles << QList<int32_t>();
						externalSurfaceTriangles[uniqueExternalSurfacesCount] << t;
						uniqueExternalSurfacesCount++;
					}

				}
			}
		}

		// We are now ready to start looping over each surface and building up a NURBS representation
		// of each surface and a NURBS curve describing the boundary of the surface
		int32_t surfcnt = -1;
		// Let's do internal surfaces first
		for (QList<QList<int32_t>>::iterator cursurf = internalSurfaceTriangles.begin(); cursurf != internalSurfaceTriangles.end(); ++cursurf)
		{
			surfcnt += 1;
			// We need to calculate the normal vector for this surface by 
			// averaging the normal vectors for each individual face.
			float surfnorm[3] = { 0.0f, 0.0f, 0.0f };

			for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
			{
				/*surfnorm[0] += m_SurfaceMeshTriangleNormals[3 * (*t) + 0];
				surfnorm[1] += m_SurfaceMeshTriangleNormals[3 * (*t) + 1];
				surfnorm[2] += m_SurfaceMeshTriangleNormals[3 * (*t) + 2];*/
				float a[3], b[3], c[3], n[3];
				triangleGeom->getVertCoordsAtTri(*t, a, b, c);
				GeometryMath::FindPlaneNormalVector(a, b, c, n);
				surfnorm[0] += n[0];
				surfnorm[1] += n[1];
				surfnorm[2] += n[2];
			}
			surfnorm[0] = surfnorm[0] / (*cursurf).size();
			surfnorm[1] = surfnorm[1] / (*cursurf).size();
			surfnorm[2] = surfnorm[2] / (*cursurf).size();

			float norm = sqrt(surfnorm[0] * surfnorm[0] + surfnorm[1] * surfnorm[1] + surfnorm[2] * surfnorm[2]);
			surfnorm[0] = surfnorm[0] / norm;
			surfnorm[1] = surfnorm[1] / norm;
			surfnorm[2] = surfnorm[2] / norm;

			// We Need A Bounding Box - The code below is equivalent to the
			// functionality of GeometryMath::FindBoundingBoxOfFaces without
			// the need to create an Int32Int32DynamicListArray
			// Matrix rotation will be used to find the actual optimal bounding
			// box (i.e. we will rotate norm to [0, 0, 1] the xy plane).

			// Calculate rotation matrix from surfnorm -> [0, 0, 1]
			float rotmat[3][3];
			float tovec[3] = { 0.0f, 0.0f, 1.0f };
			MatrixMath::TransformationMatrixFromAToB(surfnorm, tovec, rotmat);



			float ll[3], ur[3];
			ll[0] = 100000000.0;
			ll[1] = 100000000.0;
			ll[2] = 100000000.0;
			ur[0] = 0.0;
			ur[1] = 0.0;
			ur[2] = 0.0;
			for (int i = 0; i < (*cursurf).size(); ++i)
			{
				float facell[3], faceur[3];
				GeometryMath::FindBoundingBoxOfRotatedFace(triangleGeom, (*cursurf)[i], rotmat, facell, faceur);
				if (facell[0] < ll[0])
				{
					ll[0] = facell[0];
				}
				if (facell[1] < ll[1])
				{
					ll[1] = facell[1];
				}
				if (facell[2] < ll[2])
				{
					ll[2] = facell[2];
				}
				if (faceur[0] > ur[0])
				{
					ur[0] = faceur[0];
				}
				if (faceur[1] > ur[1])
				{
					ur[1] = faceur[1];
				}
				if (faceur[2] > ur[2])
				{
					ur[2] = faceur[2];
				}
			}



			float pt1[3] = { ll[0], ll[1], ll[2] };
			float pt2[3] = { ll[0], ll[1], ur[2] };
			float pt3[3] = { ll[0], ur[1], ur[2] };
			float pt4[3] = { ll[0], ur[1], ll[2] };
			float pt5[3] = { ur[0], ur[1], ur[2] };
			float pt6[3] = { ur[0], ur[1], ll[2] };
			float pt7[3] = { ur[0], ll[1], ll[2] };
			float pt8[3] = { ur[0], ll[1], ur[2] };
			float ptr1[3];
			float ptr2[3];
			float ptr3[3];
			float ptr4[3];
			float ptr5[3];
			float ptr6[3];
			float ptr7[3];
			float ptr8[3];

			float invmat[3][3];
			MatrixMath::Invert3x3(rotmat, invmat);
			MatrixMath::Transpose3x3(invmat, rotmat);
			MatrixMath::Multiply3x3with3x1(rotmat, pt1, ptr1);
			MatrixMath::Multiply3x3with3x1(rotmat, pt2, ptr2);
			MatrixMath::Multiply3x3with3x1(rotmat, pt3, ptr3);
			MatrixMath::Multiply3x3with3x1(rotmat, pt4, ptr4);
			MatrixMath::Multiply3x3with3x1(rotmat, pt5, ptr5);
			MatrixMath::Multiply3x3with3x1(rotmat, pt6, ptr6);
			MatrixMath::Multiply3x3with3x1(rotmat, pt7, ptr7);
			MatrixMath::Multiply3x3with3x1(rotmat, pt8, ptr8);

			// We now have a bounding box in which the "X-Y Plane" described by the
			// point sets 1,4,6,7 and 2,3,5,8 are the planes we need to build a grid
			// across.  Let's find the dimensions and use linspace and the coordinate
			// transform to get two sets of coordinates for ray tracing

			float xdim = ur[0] - ll[0];
			float ydim = ur[1] - ll[1];

			int32_t xcnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * xdim / ydim)));
			int32_t ycnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * ydim / xdim)));

			float *griddedsurf = new float[xcnt*ycnt * 3];


			std::vector<float> xvals = DREAM3DMath::linspace(ll[0], ur[0], xcnt);
			std::vector<float> yvals = DREAM3DMath::linspace(ll[1], ur[1], ycnt);

			// We now have a grid in the x,y plane that will need to be transformed
			// back into the normal coordinate system.  At the same time, we should
			// attempt to fit this grid with the edges.  The process will work 
			// something like this:
			//   1)  Find the edges for this surface (there may be multiple sets of 
			//       disconnected edges - need to be prepared for this possibility).
			//   2)  Find the sets of edges and build the connectivity list.  This
			//       should be a list set ordered such that pairs of vertex values
			//       are listed in order:
			//         2       6
			//         6      14
			//         14    154
			//         154    72
			//         72      2
			//   3)  A separate surface will be fit for each set.  For each set
			//       and for each point set, we will find the nearest point in our
			//       grid to the first value.  This is done in the transformed plane
			//       so that we are looking only for x and y values.  We adjust the
			//       the x,y values of the nearest point to match and set the z value
			//       offset from the plane of the bbox.  We then repeat this for the
			//       other point in the point set.  We then move the values on the 
			//       adjacent points in the set so that a line is formed between the
			//       two points.  We mark all of these points as being found and we
			//       don't search for them later.  This process repeats for all point
			//       sets.
			//   4)  For each side of the rectangular grid, we find the point closest
			//       to the edge in each row/column and move it to the outside edge of
			//       the grid.  We need to consider the possibility that there may be
			//       more than two points in each row or column.  If there are, we move
			//       those into the next adjacent spot.
			//   5)  We then perform bi-linear interpolation for all null interior
			//       points on the grid and transform our grid back into the specimen
			//       coordinate system.  We now have a search grid.  The points marked
			//       as complete will not be searched and will be copied directly to
			//       the output surface.  The points that are inside will be fit to the
			//       correct values during ray tracing.  


			// Let's find a list of vertex points that map to the edge of the
			// surface from the list of triangles.
			int64_t curfeaturefaceid = m_SurfaceMeshFeatureFaceIds[(*cursurf)[0]];
			int64_t othergrain;

			// Figure out what the other grain is for this surface for later
			if (m_SurfaceMeshFeatureFaceLabels[2 * curfeaturefaceid] == grain)
				othergrain = m_SurfaceMeshFeatureFaceLabels[2 * curfeaturefaceid + 1];
			else if (m_SurfaceMeshFeatureFaceLabels[2 * curfeaturefaceid + 1] == grain)
				othergrain = m_SurfaceMeshFeatureFaceLabels[2 * curfeaturefaceid];
			else
				assert(false);

			QList<QList<int64_t>> edgelist;
			for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
			{
				// Get the list of neighbors for this triangle
				uint16_t neighborcount = m_TriangleNeighbors->getNumberOfElements(*t);
				int64_t* nList = m_TriangleNeighbors->getElementListPointer(*t);

				// For each neighbor
				for (uint16_t i = 0; i < neighborcount; ++i)
				{
					if ((m_SurfaceMeshFaceLabels[nList[i] * 2] == grain || m_SurfaceMeshFaceLabels[nList[i] * 2 + 1] == grain) &&
						(m_SurfaceMeshFaceLabels[nList[i] * 2] != othergrain && m_SurfaceMeshFaceLabels[nList[i] * 2 + 1] != othergrain) &&
						(m_SurfaceMeshFeatureFaceIds[nList[i]] != curfeaturefaceid))
					{
						QList<int64_t> outvals;
						int64_t a[3];
						int64_t b[3];
						a[0] = triangles[3 * nList[i]];
						a[1] = triangles[3 * nList[i] + 1];
						a[2] = triangles[3 * nList[i] + 2];
						b[0] = triangles[3 * (*t)];
						b[1] = triangles[3 * (*t) + 1];
						b[2] = triangles[3 * (*t) + 2];

						for (int64_t c = 0; c < 3; ++c)
						{
							for (int64_t d = 0; d < 3; ++d)
							{
								if (a[c] == b[d])
								{
									outvals << a[c];
								}
							}
						}

						edgelist << outvals;
					}
				}
			}

			// We now have an unordered list of points - let's put them in order

			// The inside list are vertex id value pairs:  e.g. (2, 10) or (10, 14).
			// The middle list are an ordered set of these pairs making up a complete
			//   boundary ordered as detailed above.
			// The outside list contains all of the ordered sets for each complete
			//   boundary.
			QList<QList<QList<int64_t>>> orderededges;
			QVector<bool> visitededges(edgelist.size(), false);

			// Let's loop while we still have edges not matched to a surface
			while (visitededges.contains(false))
			{

				// Find the index of the first nonfinished value
				int64_t curpoint;
				for (curpoint = 0; curpoint < edgelist.size(); ++curpoint)
				{
					if (visitededges[curpoint] == false)
						break;
				}

				// We have now visited the current point
				visitededges[curpoint] = true;

				// Let's start building the list
				QList<QList<int64_t>> curborder;
				curborder << edgelist[curpoint];

				bool openedge = false;

				// Let's build connectivity
				while (curborder.last()[1] != curborder.first()[0]){

					// Candidate edges
					QList<int64_t> candidates;
					bool reversed = false;

					// Let's loop through the list and find possible next points
					for (int64_t i = 0; i < edgelist.size(); ++i)
					{
						if (visitededges[i] == true)
							continue;  // Skip this point - we've been here already

						if (edgelist[i][0] == curborder.last()[1])
							candidates << i;  // Trailing value that match the current leading value
						else if (edgelist[i][1] == curborder.last()[1])
						{
							candidates << i;
							reversed = true;
						}
					}

					if (candidates.size() == 1) // We've found exactly one match
					{
						QList<int64_t> tmplist;
						if (reversed == true)
						{
							tmplist << edgelist[candidates[0]][1];
							tmplist << edgelist[candidates[0]][0];
						}
						else
							tmplist = edgelist[candidates[0]];

						curborder << tmplist;
						visitededges[candidates[0]] = true;
					}
					else if (candidates.size() > 1) // We've found multiple matches
					{
						// We need to recursively try building a loop from this spot
						// and identify the one that completes the current loop
						// The only scenario in which this should happen
						int64_t err = RecurseOrderedEdges(edgelist, orderededges, visitededges, curborder, candidates);
						if (err < 0)
						{
							openedge = true;
							break;
						}
					}
					else
					{
						// We didn't find the next item in the list.  That's also
						// not good.  We need to thrown an exception.
						openedge = true;
						break;
					}

				}

				if (curborder.size() > 2 && !openedge) // Make sure we have an actual edge
					orderededges << curborder;

			}


			// we screwed something up above
			if (orderededges.size() > 1)
				throw OpenBoundaryException;


			/*

			// We now have ordered edges split into the appropriate bounds
			// We also already have bounds for this surface (as long as we
			// didn't find anything new above that we didn't find before
			// using triangle geometry - which shouldn't happen).  We need
			// to loop over each pair in the list and fit them into the 
			// surface.  

			int64_t perimeter = orderededges[0].size();
			
			// if the perimeter of the bounding box is the same as
			// the perimeter we calculated above - great.
			// If its greater, we need to insert points.  If it's
			// smaller, we need to expand the bounding box.

			// find the nearest point in the x,y plane to the
			// first and second values in our ordered set

			
			float firstpt[3] = { nodes[3 * orderededges[0][0][0] + 0], nodes[3 * orderededges[0][0][0] + 1], nodes[3 * orderededges[0][0][0] + 2] };
			float firstptr[3];
			MatrixMath::Multiply3x3with3x1(rotmat, firstpt, firstptr);
			float firstx = firstptr[0];
			float firsty = firstptr[1];
			int64_t firstidx = 0;

			float secondpt[3] = { nodes[3 * orderededges[0][0][1] + 0], nodes[3 * orderededges[0][0][1] + 1], nodes[3 * orderededges[0][0][1] + 2] };
			float secondptr[3];
			MatrixMath::Multiply3x3with3x1(rotmat, secondpt, secondptr);
			float secondx = secondptr[0];
			float secondy = secondptr[1];
			int64_t secondidx;


			// We now have a bounding box in which the "X-Y Plane" described by the
			// point sets 1,4,6,7 and 2,3,5,8 are the planes we need to build a grid
			// across.  Let's find the dimensions and use linspace and the coordinate
			// transform to get two sets of coordinates for ray tracing

			float xdim = ur[0] - ll[0];
			float ydim = ur[1] - ll[1];

			int32_t xcnt = ceil((perimeter / 2) / (1 + ydim / xdim)) + 1;
			int32_t ycnt = ceil((perimeter / 2) / (1 + xdim / ydim)) + 1;

			float mindist = 10000000000.0;

			// x and y vals
			std::vector<float> xvals = DREAM3DMath::linspace(ll[0], ur[0], xcnt);
			std::vector<float> yvals = DREAM3DMath::linspace(ll[1], ur[1], ycnt);
			
			VectorEdgeIterator veiter(xvals, yvals);

			int64_t mincnt = 0;
			for (int64_t cnt = 0; cnt < perimeter; ++cnt)
			{
				firstpt[0] = nodes[3 * orderededges[0][cnt][0] + 0];
				firstpt[1] = nodes[3 * orderededges[0][cnt][0] + 1];
				firstpt[2] = nodes[3 * orderededges[0][cnt][0] + 2];
				MatrixMath::Multiply3x3with3x1(rotmat, firstpt, firstptr);
				if (veiter.distance(0, firstptr[0], firstptr[1]) < mindist)
				{
					mindist = veiter.distance(0, firstptr[0], firstptr[1]);
					mincnt = cnt;
				}			
			}

			QList<QList<int64_t>> updatedlist;
			int64_t pntrcnt = mincnt;
			for (int64_t cnt = 0; cnt < perimeter; ++cnt)
			{
				updatedlist << orderededges[0][pntrcnt];
				++pntrcnt;
				if (pntrcnt >= perimeter)
					pntrcnt = 0;
			}

			/*for (int64_t cnt = 0; cnt < veiter.size(); ++cnt)
			{
				if (veiter.distance(cnt, firstx, firsty) < mindist)
				{
					mindist = veiter.distance(cnt, firstx, firsty);
					firstidx = cnt;
				}
			}**
			int64_t fwdidx = firstidx + 1;
			if (fwdidx == veiter.size())
				fwdidx = 0;
			int64_t bwdidx = firstidx - 1;
			if (bwdidx == -1)
				bwdidx = veiter.size() - 1;
			if (veiter.distance(bwdidx, secondx, secondy) < veiter.distance(fwdidx, secondx, secondy))
				secondidx = firstidx - 1;
			else
				secondidx = firstidx + 1;

			// We now have the first two points in the grid and can drop in the remaining points.
			// We can now start building the grid.

			float *griddedsurf = new float[xcnt*ycnt * 3];
			float *topbounds = new float[xcnt*ycnt * 3];
			float *botbounds = new float[xcnt*ycnt * 3];


			if (veiter.size() > perimeter)
			{
				// add interpolated points
				int64_t interpcount = veiter.size() - perimeter;
				int64_t spacing = veiter.size() / interpcount;
				int64_t addedpts = 0;
				int64_t actcnt = 0;

				// If spacing is less than or equal to 1, this code will fail.
				// It shouldn't be less than 1 (i.e. we are adding more interpolated points than real points).
				assert(spacing > 1);  

				for (int64_t cnt = 0; cnt < perimeter; ++cnt)
				{
					int64_t cursub[2];
					veiter.ind2sub(veiter.ind2gridind(firstidx, secondidx, actcnt), cursub);
					float pt[3] = { nodes[3 * orderededges[0][cnt][0] + 0], nodes[3 * orderededges[0][cnt][0] + 1], nodes[3 * orderededges[0][cnt][0] + 2] };
					float ptr[3];
					MatrixMath::Multiply3x3with3x1(rotmat, pt, ptr);
					topbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ptr[0];
					topbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ptr[1];
					topbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ur[2] + (ptr[2] - ll[2]);
					botbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ptr[0];
					botbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ptr[1];
					botbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (ll[2] - (ur[2] - ll[2])) + (ptr[2] - ll[2]);
					++actcnt;

					if (addedpts * spacing == cnt && addedpts < interpcount) {
						++addedpts;
						veiter.ind2sub(veiter.ind2gridind(firstidx, secondidx, actcnt), cursub);
						int64_t nextpt = cnt + 1;
						if (nextpt >= perimeter)
							nextpt = nextpt - perimeter;
						float npt[3] = { nodes[3 * orderededges[0][nextpt][0] + 0], nodes[3 * orderededges[0][nextpt][0] + 1], nodes[3 * orderededges[0][nextpt][0] + 2] };
						float nptr[3];
						MatrixMath::Multiply3x3with3x1(rotmat, npt, nptr);
						topbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (nptr[0] + ptr[0]) / 2;
						topbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (nptr[1] + ptr[1]) / 2;
						topbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ur[2] + (( (nptr[2] + ptr[2])/2 ) - ll[2]);
						botbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (nptr[0] + ptr[0]) / 2;
						botbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (nptr[1] + ptr[1]) / 2;
						botbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (ll[2] - (ur[2] - ll[2])) + (((nptr[2] + ptr[2]) / 2) - ll[2]);
						++actcnt;
					}
				}

			}
			else
			{
				// it's exact... just drop in points
				for (int64_t cnt = 0; cnt < perimeter; ++cnt)
				{
					int64_t cursub[2];
					veiter.ind2sub(veiter.ind2gridind(firstidx, secondidx, cnt), cursub);
					topbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = nodes[3 * orderededges[0][cnt][0] + 0];
					topbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = nodes[3 * orderededges[0][cnt][0] + 1];
					topbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = ur[2] + (nodes[3 * orderededges[0][cnt][0] + 2] - ll[2]);
					botbounds[0 + cursub[1] * 3 + cursub[0] * ycnt * 3] = nodes[3 * orderededges[0][cnt][0] + 0];
					botbounds[1 + cursub[1] * 3 + cursub[0] * ycnt * 3] = nodes[3 * orderededges[0][cnt][0] + 1];
					botbounds[2 + cursub[1] * 3 + cursub[0] * ycnt * 3] = (ll[2] - (ur[2] - ll[2])) + (nodes[3 * orderededges[0][cnt][0] + 2] - ll[2]);
				}

			}

			/*
			QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_u1_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			FILE* f = fopen(filename.toLatin1().data(), "w");
			for (int64_t i = 0; i < xcnt; i++)
			{
				char line[5000];
				int cnt = sprintf(line, "%f,%f,%f\n", topbounds[0 + 0 * 3 + i * ycnt * 3], topbounds[1 + 0 * 3 + i * ycnt * 3], topbounds[2 + 0 * 3 + i * ycnt * 3]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_u2_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (int64_t i = 0; i < xcnt; i++)
			{
				char line[5000];
				int cnt = sprintf(line, "%f,%f,%f\n", topbounds[0 + (ycnt - 1) * 3 + i * ycnt * 3], topbounds[1 + (ycnt - 1) * 3 + i * ycnt * 3], topbounds[2 + (ycnt - 1) * 3 + i * ycnt * 3]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_v1_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (int64_t i = 0; i < ycnt; i++)
			{
				char line[5000];
				int cnt = sprintf(line, "%f,%f,%f\n", topbounds[0 + i * 3 + 0 * ycnt * 3], topbounds[1 + i * 3 + 0 * ycnt * 3], topbounds[2 + i * 3 + 0 * ycnt * 3]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_v2_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (int64_t i = 0; i < ycnt; i++)
			{
				char line[5000];
				int cnt = sprintf(line, "%f,%f,%f\n", topbounds[0 + i * 3 + (xcnt - 1) * ycnt * 3], topbounds[1 + i * 3 + (xcnt - 1) * ycnt * 3], topbounds[2 + i * 3 + (xcnt - 1) * ycnt * 3]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);

			
			
			// Temporary surface output
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_BoundsRays_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");**

			// Now we can do bilinear interpolation on the grid before transforming it back to the correct frame.
			for (int64_t i = 1; i < xcnt - 2; ++i)
			{
				for (int64_t j = 1; j < ycnt - 2; ++j)
				{
					// For each inside point - let's interpolate
					float p1[3] = { topbounds[0 + j * 3 + 0 * ycnt * 3],      topbounds[1 + j * 3 + 0 * ycnt * 3],      topbounds[2 + j * 3 + 0 * ycnt * 3] };
					float p2[3] = { topbounds[0 + j * 3 + (xcnt -1) * ycnt * 3], topbounds[1 + j * 3 + (xcnt - 1) * ycnt * 3], topbounds[2 + j * 3 + (xcnt - 1) * ycnt * 3] };
					float p3[3] = { topbounds[0 + 0 * 3 + i * ycnt * 3],      topbounds[1 + 0 * 3 + i * ycnt * 3],      topbounds[2 + 0 * 3 + i * ycnt * 3] };
					float p4[3] = { topbounds[0 + (ycnt - 1) * 3 + i * ycnt * 3], topbounds[1 + (ycnt - 1) * 3 + i * ycnt * 3], topbounds[2 + (ycnt - 1) * 3 + i * ycnt * 3] };
					float p5[3] = { botbounds[0 + j * 3 + 0 * ycnt * 3],      botbounds[1 + j * 3 + 0 * ycnt * 3],      botbounds[2 + j * 3 + 0 * ycnt * 3] };
					float p6[3] = { botbounds[0 + j * 3 + (xcnt - 1) * ycnt * 3], botbounds[1 + j * 3 + (xcnt - 1) * ycnt * 3], botbounds[2 + j * 3 + (xcnt - 1) * ycnt * 3] };
					float p7[3] = { botbounds[0 + 0 * 3 + i * ycnt * 3],      botbounds[1 + 0 * 3 + i * ycnt * 3],      botbounds[2 + 0 * 3 + i * ycnt * 3] };
					float p8[3] = { botbounds[0 + (ycnt - 1) * 3 + i * ycnt * 3], botbounds[1 + (ycnt - 1) * 3 + i * ycnt * 3], botbounds[2 + (ycnt - 1) * 3 + i * ycnt * 3] };

					//char line[5000];
					//int cnt = sprintf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], p3[0], p3[1], p3[2], p4[0], p4[1], p4[2], p5[0], p5[1], p5[2], p6[0], p6[1], p6[2], p7[0], p7[1], p7[2], p8[0], p8[1], p8[2]);
					//fwrite(line, sizeof(char), cnt, f);
					
					float d1, d2, d3, d4;

					GeometryMath::FindDistanceBetweenPoints(p1, p2, d1);
					GeometryMath::FindDistanceBetweenPoints(p3, p4, d2);
					GeometryMath::FindDistanceBetweenPoints(p5, p6, d3);
					GeometryMath::FindDistanceBetweenPoints(p7, p8, d4);

					float l1 = (float)i / ((float)(xcnt - 1));
					float l2 = (float)j / ((float)(ycnt - 1));

					float np1[3] = { p1[0] + (p2[0] - p1[0])*(l1 / d1),
									 p1[1] + (p2[1] - p1[1])*(l1 / d1),
									 p1[2] + (p2[2] - p1[2])*(l1 / d1) };
					float np2[3] = { p3[0] + (p4[0] - p3[0])*(l2 / d2),
									 p3[1] + (p4[1] - p3[1])*(l2 / d2),
									 p3[2] + (p4[2] - p3[2])*(l2 / d2) };
					float np3[3] = { p5[0] + (p6[0] - p5[0])*(l1 / d3),
									 p5[1] + (p6[1] - p5[1])*(l1 / d3),
									 p5[2] + (p6[2] - p5[2])*(l1 / d3) };
					float np4[3] = { p7[0] + (p8[0] - p7[0])*(l2 / d4),
									 p7[1] + (p8[1] - p7[1])*(l2 / d4),
									 p7[2] + (p8[2] - p7[2])*(l2 / d4) };

					topbounds[0 + j * 3 + i * ycnt * 3] = (np1[0] + np2[0]) / 2;
					topbounds[1 + j * 3 + i * ycnt * 3] = (np1[1] + np2[1]) / 2;
					topbounds[2 + j * 3 + i * ycnt * 3] = (np1[2] + np2[2]) / 2;
					botbounds[0 + j * 3 + i * ycnt * 3] = (np3[0] + np4[0]) / 2;
					botbounds[1 + j * 3 + i * ycnt * 3] = (np3[1] + np4[1]) / 2;
					botbounds[2 + j * 3 + i * ycnt * 3] = (np3[2] + np4[2]) / 2;

				}
			}

			/*fclose(f);

			**
			// Temporary surface output
			QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_BoundsTop_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			FILE *f = fopen(filename.toLatin1().data(), "w");
			for (int32_t i = 0; i < xcnt; i++)
			{
				for (int32_t j = 0; j < ycnt; j++)
				{
					char line[100];
					int cnt = sprintf(line, "%f,%f,%f\n", topbounds[0 + j * 3 + i*ycnt * 3], topbounds[1 + j * 3 + i*ycnt * 3], topbounds[2 + j * 3 + i*ycnt * 3]);
					fwrite(line, sizeof(char), cnt, f);
				}
			}
			fclose(f);

			// Temporary surface output
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_BoundsBot_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (int32_t i = 0; i < xcnt; i++)
			{
				for (int32_t j = 0; j < ycnt; j++)
				{
					char line[100];
					int cnt = sprintf(line, "%f,%f,%f\n", botbounds[0 + j * 3 + i*ycnt * 3], botbounds[1 + j * 3 + i*ycnt * 3], botbounds[2 + j * 3 + i*ycnt * 3]);
					fwrite(line, sizeof(char), cnt, f);
				}
			}
			fclose(f);

			// Temporary surface output
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_Boundsllur_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			char line[100];
			int cnt = sprintf(line, "%f,%f,%f\n", ll[0], ll[1], ll[2]);
			fwrite(line, sizeof(char), cnt, f);
			cnt = sprintf(line, "%f,%f,%f\n", ur[0], ur[1], ur[2]);
			fwrite(line, sizeof(char), cnt, f);
			fclose(f);


			// One more time - loop through to invert back to correct coordinates
			float invmat[3][3];
			MatrixMath::Invert3x3(rotmat, invmat);
			MatrixMath::Transpose3x3(invmat, rotmat);
			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					float newval[3];
					MatrixMath::Multiply3x3with3x1(rotmat, &topbounds[j * 3 + i * ycnt * 3], newval);
					topbounds[0 + j * 3 + i * ycnt * 3] = newval[0];
					topbounds[1 + j * 3 + i * ycnt * 3] = newval[1];
					topbounds[2 + j * 3 + i * ycnt * 3] = newval[2];
					MatrixMath::Multiply3x3with3x1(rotmat, &botbounds[j * 3 + i * ycnt * 3], newval);
					botbounds[0 + j * 3 + i * ycnt * 3] = newval[0];
					botbounds[1 + j * 3 + i * ycnt * 3] = newval[1];
					botbounds[2 + j * 3 + i * ycnt * 3] = newval[2];
				}
			}

			*/


			// We might consider marking triangles as found or not found
			// and then repeat ray tracing on the unfound triangles until
			// all triangles have been found.  This should result in the
			// normal shifting slightly.  We would have to keep the same
			// bounding box, it would just be sitting at a different angle.


			// Let's do ray tracing in parallel
			QString ss = QObject::tr("Ray Tracing In Progress...");
			notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);
			#ifdef DREAM3D_USE_PARALLEL_ALGORITHMS
			if (doParallel == true)
			{
				tbb::parallel_for(tbb::blocked_range<int64_t>(0, xcnt),
					RayTraceSurf(xcnt, ycnt, xvals, yvals, rotmat, ll, ur,
					triangleGeom, cursurf, griddedsurf, m_SurfaceMeshTriangleNormals,
					this), tbb::auto_partitioner());
			}
			else
			#endif
			{
				RayTraceSurf serial(xcnt, ycnt, xvals, yvals, rotmat, ll, ur,
					triangleGeom, cursurf, griddedsurf, m_SurfaceMeshTriangleNormals, this);
				serial.generate(0, xcnt, xcnt, ycnt, xvals, yvals, rotmat, ll, ur,
					triangleGeom, cursurf, griddedsurf, m_SurfaceMeshTriangleNormals, this);
			}
			if (getCancel() == true) { return; }

			
			// Temporary surface output
			QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("SurfaceBeforeEdges_") + QString::number(surfcnt) + QString("_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			FILE *f = fopen(filename.toLatin1().data(), "w");
			for (int32_t i = 0; i < xcnt; i++)
			{
				for (int32_t j = 0; j < ycnt; j++)
				{
					char line[100];
					int cnt = sprintf(line, "%f,%f,%f\n", griddedsurf[0 + j * 3 + i*ycnt * 3], griddedsurf[1 + j * 3 + i*ycnt * 3], griddedsurf[2 + j * 3 + i*ycnt * 3]);
					fwrite(line, sizeof(char), cnt, f);
				}
			}
			fclose(f);			

			


			// A Better Solution For Edge Matching - 
			// For each point in the edge, find the ray that is closest to
			// that point.  The point in the grid corresponding to that ray
			// takes the value of that point.  We record these points and
			// then ensure we have neighbor connectivity between them around
			// the entire grid - interpolating where necessary.
			// This should eliminate all of the problems associated with
			// missing edges.  This also provides the basis needed to fill in
			// the remaining missing grid points, ensuring that for every row
			// and column in the grid, we have a corresponding minimium and
			// maximum point in that row or column that matches with an edge.

			// One pitfall of this (which is also a pitfall of the method below)
			// is that the grid points end up not falling where they should 
			// naturally, being forced to match up with the nearest point.  This
			// will likely result in a twist in the grid at the edge forcing it
			// to line up with these points.  This probably isn't the end of the
			// world.


			ss = QObject::tr("Matching Edges...");
			notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);

			QList<int64_t> edgexindex;
			QList<int64_t> edgeyindex;

			QVector<bool> visitedgridpt(xcnt*ycnt, false);

			for (QList<QList<int64_t>>::iterator edge = orderededges[0].begin(); edge != orderededges[0].end(); ++edge)
			{

				int64_t x = 0;
				int64_t y = 0;
				float minvalue = 10000000000.0f;
				float curpoint[3] = { nodes[3 * (*edge)[0] + 0], nodes[3 * (*edge)[0] + 1], nodes[3 * (*edge)[0] + 2] };
				float d = surfnorm[0] * curpoint[0] + surfnorm[1] * curpoint[1] + surfnorm[2] * curpoint[2];
				float oldq[3], oldr[3], q[3], r[3];
				oldq[2] = ll[2];
				oldr[2] = ur[2];
				float p[3];
				int m;
				char retval;
				float distance;

				for (int64_t i = 0; i < xcnt; ++i)
				{
					for (int64_t j = 0; j < ycnt; ++j)
					{
						// Let's first make sure this ray isn't already
						// assigned to another edge point - this should
						// effectively result in the next closest point
						// becoming the right value
						if (!visitedgridpt[y + x*ycnt])
						{
							oldq[0] = xvals[i];
							oldq[1] = yvals[j];
							oldr[0] = xvals[i];
							oldr[1] = yvals[j];
							MatrixMath::Multiply3x3with3x1(rotmat, oldq, q);
							MatrixMath::Multiply3x3with3x1(rotmat, oldr, r);
							retval = GeometryMath::RayIntersectsPlane(surfnorm, d, q, r, p, m);
							if (retval == '1' || retval == 'q' || retval == 'r')
							{
								GeometryMath::FindDistanceBetweenPoints(curpoint, p, distance);
								if (distance < minvalue)
								{
									x = i;
									y = j;
									minvalue = distance;
								}
							}
						}						
					}
				}

				griddedsurf[0 + y * 3 + x*ycnt * 3] = curpoint[0];
				griddedsurf[1 + y * 3 + x*ycnt * 3] = curpoint[1];
				griddedsurf[2 + y * 3 + x*ycnt * 3] = curpoint[2];

				// Let's disable this temporarily because it seems to
				// have an adverse effect.  This could suggest that
				// our grid potentially has bad points from using this
				// method.
				//visitedgridpt[y + x*ycnt] = true;
					
				edgexindex << x;
				edgeyindex << y;

			}

			// We've found exact points in the grid for each edge vertex
			// Now we need to verify that there is connectivity between
			// the edges.  We loop over each edge pair and get the index
			// values cooresponding to the point in the grid.  We then
			// find the path between the two points and if the distance
			// between them is greater than 1, we fill in the points
			// along the shortest distance with the appropriate
			// interpolated values.

			for (QList<QList<int64_t>>::iterator edge = orderededges[0].begin(); edge != orderededges[0].end(); ++edge)
			{
				float q[3] = { nodes[3 * (*edge)[0] + 0], nodes[3 * (*edge)[0] + 1], nodes[3 * (*edge)[0] + 2] };
				float r[3] = { nodes[3 * (*edge)[1] + 0], nodes[3 * (*edge)[1] + 1], nodes[3 * (*edge)[1] + 2] };

				size_t curcount = edge - orderededges[0].begin();
				size_t nextcount = curcount + 1;
				if (nextcount == orderededges[0].size())
					nextcount = 0;

				int64_t a[2] = { edgexindex[curcount], edgeyindex[curcount] };
				int64_t b[2] = { edgexindex[nextcount], edgeyindex[nextcount] };

				// We now use Bresenham's algorithm to find the points to interpolate
				// Algorithm modified from code presented on rosettacode.org and
				// released under GFDL
				
				int64_t x0 = a[0], x1 = b[0];
				int64_t y0 = a[1], y1 = b[1];
				int64_t dx = abs(x1 - x0), sx = x0<x1 ? 1 : -1;
				int64_t dy = abs(y1 - y0), sy = y0<y1 ? 1 : -1;
				int64_t err = (dx>dy ? dx : -dy) / 2, e2;
				QList<int64_t> interpxpoints;
				QList<int64_t> interpypoints;

				for (;;){
					interpxpoints << x0;
					interpypoints << y0;
					if (x0 == x1 && y0 == y1) break;
					e2 = err;
					if (e2 >-dx) { err -= dy; x0 += sx; }
					if (e2 < dy) { err += dx; y0 += sy; }
				}

				float interplength = (float)(interpxpoints.size());

				if (interpxpoints.size() > 2)
				{
					float lc[3];
					lc[0] = r[0] - q[0];
					lc[1] = r[1] - q[1];
					lc[2] = r[2] - q[2];

					for (int64_t cnt = 1; cnt < interpxpoints.size() - 1; ++cnt)
					{
						float frac = (float)(cnt + 1) / interplength;
						griddedsurf[0 + interpypoints[cnt] * 3 + interpxpoints[cnt] * ycnt * 3] = lc[0] * frac + q[0];
						griddedsurf[1 + interpypoints[cnt] * 3 + interpxpoints[cnt] * ycnt * 3] = lc[1] * frac + q[1];
						griddedsurf[2 + interpypoints[cnt] * 3 + interpxpoints[cnt] * ycnt * 3] = lc[2] * frac + q[2];
					}

				}

			}



			

			// Great!  Now we've managed to fit the grid down onto the surface
			// However, there are very likely still interior points that weren't
			// found if the triangle at that point happens to be almost perfectly
			// parallel to the ray.  We should have enough resolution that all we
			// need to do is interpolate the grid and call it good.  We also have
			// the problem of dealing with the surface outside of the edges.

			// Interpolation needs to consider where the original ray was actually
			// located at.  Interpolation on the outside of the grid needs to 
			// progress outwards by choosing the point on the ray that falls in the
			// same plane as the end point (rather than just taking the edge value
			// as is).  Internal interpolation 'probably' needs to do the same thing
			// to keep everything monotonic.

			// Let's use bilinear interpolation or nearest neighbor blended as appropriate
			ss = QObject::tr("Filling Missing Grid Values...");
			notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);

			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					// Is the current point unset
					// Only need to check one component since there are no scenarios in which the three values could be changed separately
					if (isnan(griddedsurf[0 + j * 3 + i * ycnt * 3])) 
					{
						float x1[3] = { nanf(""), nanf(""), nanf("") };
						int64_t x1idx = i;
						float x2[3] = { nanf(""), nanf(""), nanf("") };
						int64_t x2idx = i;
						float y1[3] = { nanf(""), nanf(""), nanf("") };
						int64_t y1idx = j;
						float y2[3] = { nanf(""), nanf(""), nanf("") };
						int64_t y2idx = j;
						
						float xinterp[3] = { 0.0f, 0.0f, 0.0f };
						float yinterp[3] = { 0.0f, 0.0f, 0.0f };

						float xfrac = 0.0f;
						float yfrac = 0.0f;

						int64_t curidx = i;

						// Let's find each component

						// Left X Direction
						while (true)
						{
							curidx -= 1;
							if (curidx < 0)
								break;
							if (!isnan(griddedsurf[0 + j * 3 + curidx * ycnt * 3]))
							{
								x1idx = curidx;
								x1[0] = griddedsurf[0 + j * 3 + curidx * ycnt * 3];
								x1[1] = griddedsurf[1 + j * 3 + curidx * ycnt * 3];
								x1[2] = griddedsurf[2 + j * 3 + curidx * ycnt * 3];
							}
						}
						curidx = i;

						// Right X Direction
						while (true)
						{
							curidx += 1;
							if (curidx == xcnt)
								break;
							if (!isnan(griddedsurf[0 + j * 3 + curidx * ycnt * 3]))
							{
								x2idx = curidx;
								x2[0] = griddedsurf[0 + j * 3 + curidx * ycnt * 3];
								x2[1] = griddedsurf[1 + j * 3 + curidx * ycnt * 3];
								x2[2] = griddedsurf[2 + j * 3 + curidx * ycnt * 3];
							}
						}
						curidx = j;

						// Up Y Direction
						while (true)
						{
							curidx -= 1;
							if (curidx < 0)
								break;
							if (!isnan(griddedsurf[0 + curidx * 3 + i * ycnt * 3]))
							{
								y1idx = curidx;
								y1[0] = griddedsurf[0 + curidx * 3 + i * ycnt * 3];
								y1[1] = griddedsurf[1 + curidx * 3 + i * ycnt * 3];
								y1[2] = griddedsurf[2 + curidx * 3 + i * ycnt * 3];
							}
						}
						curidx = j;

						// Down Y Direction
						while (true)
						{
							curidx += 1;
							if (curidx == ycnt)
								break;
							if (!isnan(griddedsurf[0 + curidx * 3 + i * ycnt * 3]))
							{
								y2idx = curidx;
								y2[0] = griddedsurf[0 + curidx * 3 + i * ycnt * 3];
								y2[1] = griddedsurf[1 + curidx * 3 + i * ycnt * 3];
								y2[2] = griddedsurf[2 + curidx * 3 + i * ycnt * 3];
							}
						}


						// Now, figure out which type of interpolation to use
						if (x1idx == i && x2idx == i && y1idx == j && y2idx == j)
						{
							// we need to skip this point for now because it lies 
							// at a point in which we don't yet know how to
							// interpolate it.  We will need to loop through again
							// and fix this.

							// assert(false);

						}
						else if ((((x1idx != i) + (x2idx != i)) + ((y1idx != j) + (y2idx != j)) == 1))
						{
							// We know one value in one direction only.  This should
							// happen at every point along the outside of the grid 
							// except for the corners.  We just set this to be that
							// one value.

							float pt[3];

							if (x1idx != i)
							{
								pt[0] = x1[0];
								pt[1] = x1[1];
								pt[2] = x1[2];
							}
							else if (x2idx != i)
							{
								pt[0] = x2[0];
								pt[1] = x2[1];
								pt[2] = x2[2];
							}
							else if (y1idx != j)
							{
								pt[0] = y1[0];
								pt[1] = y1[1];
								pt[2] = y1[2];
							}
							else if (y2idx != j)
							{
								pt[0] = y2[0];
								pt[1] = y2[1];
								pt[2] = y2[2];
							}

							float oldq[3], oldr[3], q[3], r[3];
							float p[3];
							int m;
							char retval;
							float d = surfnorm[0] * pt[0] + surfnorm[1] * pt[1] + surfnorm[2] * pt[2];
							oldq[0] = xvals[i];
							oldq[1] = yvals[j];
							oldq[2] = ll[2];
							oldr[0] = xvals[i];
							oldr[1] = yvals[j];
							oldr[2] = ur[2];
							MatrixMath::Multiply3x3with3x1(rotmat, oldq, q);
							MatrixMath::Multiply3x3with3x1(rotmat, oldr, r);
							retval = GeometryMath::RayIntersectsPlane(surfnorm, d, q, r, p, m);
							assert(retval == '1' || retval == 'q' || retval == 'r');
							
							griddedsurf[0 + j * 3 + i * ycnt * 3] = p[0];
							griddedsurf[1 + j * 3 + i * ycnt * 3] = p[1];
							griddedsurf[2 + j * 3 + i * ycnt * 3] = p[2];

							
						}
						else if (((x1idx != i) ^ (x2idx != i)) && ((y1idx != j) ^ (y2idx != j)))
						{
							// We have one value in each direction - this should
							// only happen during the second iteration to fill in
							// the gaps described by the scenario above which occurs
							// at the corners of the grid.

							// If we find any of these on the first loop through, it
							// means there is still a giant hole in the grid.  Let's
							// skip it for now and see if we can improve the fit on
							// the second loop through

							/*
							
							if (x1idx != i)
							{
								xinterp[0] = x1[0];
								xinterp[1] = x1[1];
								xinterp[2] = x1[2];
							}
							else
							{
								xinterp[0] = x2[0];
								xinterp[1] = x2[1];
								xinterp[2] = x2[2];
							}
							if (y1idx != j)
							{
								yinterp[0] = y1[0];
								yinterp[1] = y1[1];
								yinterp[2] = y1[2];
							}
							else
							{
								yinterp[0] = y2[0];
								yinterp[1] = y2[1];
								yinterp[2] = y2[2];
							}

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
							
							*/


						}
						else if (x1idx != i && x2idx != i && y1idx == j && y2idx == j)
						{
							// We have two values in the x direction
							// This suggests a an entire column that is missing
							// Let's just average this between the adjacent columns
							griddedsurf[0 + j * 3 + i * ycnt * 3] = (x1[0] + x2[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (x1[1] + x2[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (x1[2] + x2[2]) / 2;
						}
						else if (x1idx == i && x2idx == i && y1idx != j && y2idx != j)
						{
							// We have two values in the y direction
							// This suggests a an entire row that is missing
							// Let's just average this between the adjacent rows
							griddedsurf[0 + j * 3 + i * ycnt * 3] = (y1[0] + y2[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (y1[1] + y2[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (y1[2] + y2[2]) / 2;
						}
						/*else if (x1idx == i && x2idx != i && y1idx != j && y2idx != j)
						{
							// We have values to the right, top, and bottom
							// Let's do a blend between the single value of x
							// and the linearly interpolated value of y
							xinterp[0] = x2[0];
							xinterp[1] = x2[1];
							xinterp[2] = x2[2];
							
							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = y1[0] + (y2[0] - y1[0])*yfrac;
							yinterp[1] = y1[1] + (y2[1] - y1[1])*yfrac;
							yinterp[2] = y1[2] + (y2[2] - y1[2])*yfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx == i && y1idx != j && y2idx != j)
						{
							// We have values to the left, top, and bottom
							// Let's do a blend between the single value of x
							// and the linearly interpolated value of y
							xinterp[0] = x1[0];
							xinterp[1] = x1[1];
							xinterp[2] = x1[2];

							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = y1[0] + (y2[0] - y1[0])*yfrac;
							yinterp[1] = y1[1] + (y2[1] - y1[1])*yfrac;
							yinterp[2] = y1[2] + (y2[2] - y1[2])*yfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx != i && y1idx == j && y2idx != j)
						{
							// We have values to the left, right, and bottom
							yinterp[0] = y2[0];
							yinterp[1] = y2[1];
							yinterp[2] = y2[2];

							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = x1[0] + (x2[0] - x1[0])*xfrac;
							xinterp[1] = x1[1] + (x2[1] - x1[1])*xfrac;
							xinterp[2] = x1[2] + (x2[2] - x1[2])*xfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx != i && y1idx != j && y2idx == j)
						{
							// We have values to the left, right, and top
							yinterp[0] = y1[0];
							yinterp[1] = y1[1];
							yinterp[2] = y1[2];

							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = x1[0] + (x2[0] - x1[0])*xfrac;
							xinterp[1] = x1[1] + (x2[1] - x1[1])*xfrac;
							xinterp[2] = x1[2] + (x2[2] - x1[2])*xfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}*/
						else if (x1idx != i && x2idx != i && y1idx != j && y2idx != j)
						{
							// We have four values - we can do bilinear interpolation
							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = (1 - xfrac) * x1[0] + (xfrac) * x2[0];
							xinterp[1] = (1 - xfrac) * x1[1] + (xfrac)* x2[1];
							xinterp[2] = (1 - xfrac) * x1[2] + (xfrac)* x2[2];

							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = (1 - yfrac) * y1[0] + (yfrac)* y2[0];
							yinterp[1] = (1 - yfrac) * y1[1] + (yfrac)* y2[1];
							yinterp[2] = (1 - yfrac) * y1[2] + (yfrac)* y2[2];

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;

							
							


						}						

					}
				}
			}


			// Let's do it one more time to take care of the corners and any stragglers
			/*
			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					// Is the current point unset
					// Only need to check one component since there are no scenarios in which the three values could be changed separately
					if (isnan(griddedsurf[0 + j * 3 + i * ycnt * 3]))
					{
						float x1[3] = { nanf(""), nanf(""), nanf("") };
						int64_t x1idx = i;
						float x2[3] = { nanf(""), nanf(""), nanf("") };
						int64_t x2idx = i;
						float y1[3] = { nanf(""), nanf(""), nanf("") };
						int64_t y1idx = j;
						float y2[3] = { nanf(""), nanf(""), nanf("") };
						int64_t y2idx = j;

						float xinterp[3] = { 0.0f, 0.0f, 0.0f };
						float yinterp[3] = { 0.0f, 0.0f, 0.0f };

						float xfrac = 0.0f;
						float yfrac = 0.0f;

						int64_t curidx = i;

						// Let's find each component

						// Left X Direction
						while (true)
						{
							curidx -= 1;
							if (curidx < 0)
								break;
							if (!isnan(griddedsurf[0 + j * 3 + curidx * ycnt * 3]))
							{
								x1idx = curidx;
								x1[0] = griddedsurf[0 + j * 3 + curidx * ycnt * 3];
								x1[1] = griddedsurf[1 + j * 3 + curidx * ycnt * 3];
								x1[2] = griddedsurf[2 + j * 3 + curidx * ycnt * 3];
							}
						}
						curidx = i;

						// Right X Direction
						while (true)
						{
							curidx += 1;
							if (curidx == xcnt)
								break;
							if (!isnan(griddedsurf[0 + j * 3 + curidx * ycnt * 3]))
							{
								x2idx = curidx;
								x2[0] = griddedsurf[0 + j * 3 + curidx * ycnt * 3];
								x2[1] = griddedsurf[1 + j * 3 + curidx * ycnt * 3];
								x2[2] = griddedsurf[2 + j * 3 + curidx * ycnt * 3];
							}
						}
						curidx = j;

						// Up Y Direction
						while (true)
						{
							curidx -= 1;
							if (curidx < 0)
								break;
							if (!isnan(griddedsurf[0 + curidx * 3 + i * ycnt * 3]))
							{
								y1idx = curidx;
								y1[0] = griddedsurf[0 + curidx * 3 + i * ycnt * 3];
								y1[1] = griddedsurf[1 + curidx * 3 + i * ycnt * 3];
								y1[2] = griddedsurf[2 + curidx * 3 + i * ycnt * 3];
							}
						}
						curidx = j;

						// Down Y Direction
						while (true)
						{
							curidx += 1;
							if (curidx == ycnt)
								break;
							if (!isnan(griddedsurf[0 + curidx * 3 + i * ycnt * 3]))
							{
								y2idx = curidx;
								y2[0] = griddedsurf[0 + curidx * 3 + i * ycnt * 3];
								y2[1] = griddedsurf[1 + curidx * 3 + i * ycnt * 3];
								y2[2] = griddedsurf[2 + curidx * 3 + i * ycnt * 3];
							}
						}


						// Now, figure out which type of interpolation to use
						if (x1idx == i && x2idx == i && y1idx == j && y2idx == j)
						{
							// we need to skip this point for now because it lies 
							// at a point in which we don't yet know how to
							// interpolate it.  We will need to loop through again
							// and fix this.

							assert(false);

						}
						else if (((x1idx != i) + (x2idx != i)) + ((y1idx != j) + (y2idx != j)) == 1)
						{
							// We know one value in one direction only.  This should
							// happen at every point along the outside of the grid 
							// except for the corners.  We just set this to be that
							// one value.

							float pt[3];

							if (x1idx != i)
							{
								pt[0] = x1[0];
								pt[1] = x1[1];
								pt[2] = x1[2];
							}
							else if (x2idx != i)
							{
								pt[0] = x2[0];
								pt[1] = x2[1];
								pt[2] = x2[2];
							}
							else if (y1idx != j)
							{
								pt[0] = y1[0];
								pt[1] = y1[1];
								pt[2] = y1[2];
							}
							else if (y2idx != j)
							{
								pt[0] = y2[0];
								pt[1] = y2[1];
								pt[2] = y2[2];
							}

							float oldq[3], oldr[3], q[3], r[3];
							float p[3];
							int m;
							char retval;
							float d = surfnorm[0] * pt[0] + surfnorm[1] * pt[1] + surfnorm[2] * pt[2];
							oldq[0] = xvals[i];
							oldq[1] = yvals[j];
							oldq[2] = ll[2];
							oldr[0] = xvals[i];
							oldr[1] = yvals[j];
							oldr[2] = ur[2];
							MatrixMath::Multiply3x3with3x1(rotmat, oldq, q);
							MatrixMath::Multiply3x3with3x1(rotmat, oldr, r);
							retval = GeometryMath::RayIntersectsPlane(surfnorm, d, q, r, p, m);
							assert(retval == '1' || retval == 'q' || retval == 'r');

							griddedsurf[0 + j * 3 + i * ycnt * 3] = p[0];
							griddedsurf[1 + j * 3 + i * ycnt * 3] = p[1];
							griddedsurf[2 + j * 3 + i * ycnt * 3] = p[2];
						}
						else if (((x1idx != i) ^ (x2idx != i)) && ((y1idx != j) ^ (y2idx != j)))
						{
							// We have one value in each direction - this should
							// only happen during the second iteration to fill in
							// the gaps described by the scenario above which occurs
							// at the corners of the grid.

							// If we find any of these on the first loop through, it
							// means there is still a giant hole in the grid.  Let's
							// skip it for now and see if we can improve the fit on
							// the second loop through

							

							if (x1idx != i)
							{
							xinterp[0] = x1[0];
							xinterp[1] = x1[1];
							xinterp[2] = x1[2];
							}
							else
							{
							xinterp[0] = x2[0];
							xinterp[1] = x2[1];
							xinterp[2] = x2[2];
							}
							if (y1idx != j)
							{
							yinterp[0] = y1[0];
							yinterp[1] = y1[1];
							yinterp[2] = y1[2];
							}
							else
							{
							yinterp[0] = y2[0];
							yinterp[1] = y2[1];
							yinterp[2] = y2[2];
							}

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;

							
						}
						else if (x1idx != i && x2idx != i && y1idx == j && y2idx == j)
						{
							// We have two values in the x direction
							// This suggests a an entire column that is missing
							// Let's just average this between the adjacent columns
							griddedsurf[0 + j * 3 + i * ycnt * 3] = (x1[0] + x2[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (x1[1] + x2[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (x1[2] + x2[2]) / 2;
						}
						else if (x1idx == i && x2idx == i && y1idx != j && y2idx != j)
						{
							// We have two values in the y direction
							// This suggests a an entire row that is missing
							// Let's just average this between the adjacent rows
							griddedsurf[0 + j * 3 + i * ycnt * 3] = (y1[0] + y2[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (y1[1] + y2[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (y1[2] + y2[2]) / 2;
						}
						else if (x1idx == i && x2idx != i && y1idx != j && y2idx != j)
						{
							// We have values to the right, top, and bottom
							// Let's do a blend between the single value of x
							// and the linearly interpolated value of y
							xinterp[0] = x2[0];
							xinterp[1] = x2[1];
							xinterp[2] = x2[2];

							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = y1[0] + (y2[0] - y1[0])*yfrac;
							yinterp[1] = y1[1] + (y2[1] - y1[1])*yfrac;
							yinterp[2] = y1[2] + (y2[2] - y1[2])*yfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx == i && y1idx != j && y2idx != j)
						{
							// We have values to the left, top, and bottom
							// Let's do a blend between the single value of x
							// and the linearly interpolated value of y
							xinterp[0] = x1[0];
							xinterp[1] = x1[1];
							xinterp[2] = x1[2];

							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = y1[0] + (y2[0] - y1[0])*yfrac;
							yinterp[1] = y1[1] + (y2[1] - y1[1])*yfrac;
							yinterp[2] = y1[2] + (y2[2] - y1[2])*yfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx != i && y1idx == j && y2idx != j)
						{
							// We have values to the left, right, and bottom
							yinterp[0] = y2[0];
							yinterp[1] = y2[1];
							yinterp[2] = y2[2];

							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = x1[0] + (x2[0] - x1[0])*xfrac;
							xinterp[1] = x1[1] + (x2[1] - x1[1])*xfrac;
							xinterp[2] = x1[2] + (x2[2] - x1[2])*xfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx != i && y1idx != j && y2idx == j)
						{
							// We have values to the left, right, and top
							yinterp[0] = y1[0];
							yinterp[1] = y1[1];
							yinterp[2] = y1[2];

							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = x1[0] + (x2[0] - x1[0])*xfrac;
							xinterp[1] = x1[1] + (x2[1] - x1[1])*xfrac;
							xinterp[2] = x1[2] + (x2[2] - x1[2])*xfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;
						}
						else if (x1idx != i && x2idx != i && y1idx != j && y2idx != j)
						{
							// We have four values - we can do bilinear interpolation
							xfrac = (float)(i - x1idx) / (float)(x2idx - x1idx);

							xinterp[0] = x1[0] + (x2[0] - x1[0])*xfrac;
							xinterp[1] = x1[1] + (x2[1] - x1[1])*xfrac;
							xinterp[2] = x1[2] + (x2[2] - x1[2])*xfrac;

							yfrac = (float)(j - y1idx) / (float)(y2idx - y1idx);

							yinterp[0] = y1[0] + (y2[0] - y1[0])*yfrac;
							yinterp[1] = y1[1] + (y2[1] - y1[1])*yfrac;
							yinterp[2] = y1[2] + (y2[2] - y1[2])*yfrac;

							griddedsurf[0 + j * 3 + i * ycnt * 3] = (xinterp[0] + yinterp[0]) / 2;
							griddedsurf[1 + j * 3 + i * ycnt * 3] = (xinterp[1] + yinterp[1]) / 2;
							griddedsurf[2 + j * 3 + i * ycnt * 3] = (xinterp[2] + yinterp[2]) / 2;

						}

					}
				}
			}

			*/

			// Temporary surface output
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("SurfaceAfterEdges_") + QString::number(surfcnt) + QString("_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (int32_t i = 0; i < xcnt; i++)
			{
				for (int32_t j = 0; j < ycnt; j++)
				{
					char line[100];
					int cnt = sprintf(line, "%f,%f,%f\n", griddedsurf[0 + j * 3 + i*ycnt * 3], griddedsurf[1 + j * 3 + i*ycnt * 3], griddedsurf[2 + j * 3 + i*ycnt * 3]);
					fwrite(line, sizeof(char), cnt, f);
				}
			}
			fclose(f);

			QMap<int64_t, int32_t> nodelist;
			for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
			{
				nodelist.insert(triangles[3 * (*t) + 0], 0);
				nodelist.insert(triangles[3 * (*t) + 1], 0);
				nodelist.insert(triangles[3 * (*t) + 2], 0);
			}
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("SurfacePoints_") + QString::number(surfcnt) + QString("_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (QMap<int64_t, int32_t>::iterator n = nodelist.begin(); n != nodelist.end(); ++n)
			{
				int64_t curnode = n.key();
				float a[3];
				triangleGeom->getCoords(curnode, a);
				char line[100];
				int cnt = sprintf(line, "%f,%f,%f\n", a[0], a[1], a[2]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);


			/*  This was the original approach below - not completed.
			    Code kept for reference.

			// Now we have the edge in order and we've found as much of the
			// surface as we can.  So, let's start the process of matching
			// up grid points with the edges.

			// Because we will have to go through multiple rounds of iterating
			// until we find the point we are interested in, we will write a
			// function that determines the next iterator index value by 
			// searching out from the current point.  Thus, we only have to loop
			// through a small number of points rather than the entire grid.
			// It might also be useful to have a quick function that converts
			// between x,y,z values and index values in the grid.
			IndexConverter idc(xcnt, ycnt, 3);

			// We first need a starting point in the grid... a point we can
			// for sure match with a found point.  
			float minvalue = 10000000000.0f;
			int64_t minindex = -1;
			int64_t gridminindex = -1;
			for (QList<QList<int64_t>>::iterator curborder = orderededges[0].begin(); curborder != orderededges[0].end(); ++curborder)
			{
				float curminvalue = 1000000000.0f;
				int64_t curminindex = -1;
				float borderpt[3] = { nodes[3 * (*curborder)[0] + 0], nodes[3 * (*curborder)[0] + 1], nodes[3 * (*curborder)[0] + 2] };
				// For every point in the edge list, determine the minimum distance
				for (int64_t x = 0; x < xcnt; ++x)
				{
					for (int64_t y = 0; y < ycnt; ++y)
					{
						int64_t curidx = idc.sub2ind(x, y, 0);
						float gridpt[3] = { griddedsurf[curidx], griddedsurf[curidx + 1], griddedsurf[curidx + 2] };
						float curdist; 
						GeometryMath::FindDistanceBetweenPoints(borderpt, gridpt, curdist);
						if (curdist < curminvalue)
						{
							curminvalue = curdist;
							curminindex = curidx;
						}
					}
				}

				if (curminvalue < minvalue)
				{
					minvalue = curminvalue;
					minindex = curborder - orderededges[0].begin();
					gridminindex = curminindex;
				}
			}

			// We might then consider reordering our edgelist to simplify
			// make things a bit easier in a moment.
			QList<QList<int64_t>> tmplist;
			int64_t edgecnt = orderededges[0].size();
			for (int64_t i = 0; i < edgecnt; ++i)
			{
				tmplist << orderededges[0][minindex];
				++minindex;
				if (minindex == edgecnt)
					minindex = 0;
			}
			orderededges[0] = tmplist;

			// Let's keep track of the previous point coordinates in the grid
			QList<int64_t> edgeindex;

			// Let's do the first point to prepare for the loop below
			griddedsurf[gridminindex] = nodes[3 * orderededges[0][0][0] + 0];
			griddedsurf[gridminindex + 1] = nodes[3 * orderededges[0][0][0] + 1];
			griddedsurf[gridminindex + 2] = nodes[3 * orderededges[0][0][0] + 2];
			edgeindex << gridminindex;


			// Looping through each edge pair, we start with the current point
			// as determined from the previous iteration and try to find 
			// the location that the next point is located.  If the next point
			// is an immediate neighbor, we move on.  If it isn't, we need to
			// interpolate between, using an algorithm to find the shortest path
			// through the grid between the two points and then fill in the values.

			float tolerance = 0.2; // Should be set to step size or less

			for (QList<QList<int64_t>>::iterator curedge = orderededges[0].begin(); curedge != orderededges[0].end(); ++curedge)
			{

				float edgept[3];

				int64_t startx, starty, startz;

				int64_t curx(startx), cury(starty);
				int64_t curidx = 0;

				size_t i = 0;

				// We need another for loop here so that we keep incrementing until we have found
				// the next edge point in case there are gaps in the edges
				for (i = 0; i < orderededges[0].size() - (curedge - orderededges[0].begin()); ++i)
				{

					//We just found (*curedge)[0], so let's try to find (*curedge)[1]
					edgept[0] = nodes[3 * (*(curedge + i))[1] + 0];
					edgept[1] = nodes[3 * (*(curedge + i))[1] + 1];
					edgept[2] = nodes[3 * (*(curedge + i))[1] + 2];

					idc.ind2sub(edgeindex[(curedge + i) - orderededges[0].begin()], startx, starty, startz);

					while (true)
					{
						// Keep looping until we get curidx = -1 or we are inside tolerance

						// Get iterator index
						curidx = idc.radit(startx, starty, curx, cury);

						if (curidx == -1)
						{
							break;
						}

						// Get distance to next point
						float gridpt[3] = { griddedsurf[curidx], griddedsurf[curidx + 1], griddedsurf[curidx + 2] };
						float curdist;
						GeometryMath::FindDistanceBetweenPoints(edgept, gridpt, curdist);

						if (curdist < tolerance)
						{
							break;
						}

					}

					if (curidx != -1)
					{
						break;
					}

				}


				// In both scenarios below, we need to handle interpolation
				if (i == 0)
				{
					// we found the next point... is it an immediate neighbor
					// if so, drop it in and continue.  otherwise, find the
					// shortest path and interpolate the points in between

				}
				else
				{
					// We are missing points on the edge suggesting a hole
					// We need to fit these points into the grid.  We need
					// to find a path through the grid to fit these points.
					// Since we are increasing the resolution of the grid
					// from the original mesh, there should be fewer missing
					// edge points than grid points not defined.
					// The generic solution to this problem is probably to
					// curve fit and then resample to fill in the shortest
					// distance in the grid.  

					// A better solution might be to look at the distance
					// between each point that is missing and the rays
					// used to find the grid.
				}

				

				

			}

			*/

			




			// Temporary edge output
			/*filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("SurfaceEdge_") + QString::number(surfcnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (QMap<int64_t, int32_t>::iterator i = edgevertexmap.begin(); i != edgevertexmap.end(); ++i)
			{
				int64_t idx = i.key();
				float a = static_cast<float>(nodes[idx * 3]);
				float b = static_cast<float>(nodes[idx * 3 + 1]);
				float c = static_cast<float>(nodes[idx * 3 + 2]);
				char line[100];
				int cnt = sprintf(line, "%f,%f,%f\n", a, b, c);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);*/






			delete[] griddedsurf;
			//delete[] topbounds;
			//delete[] botbounds;

			/***********
			// The code below attempts to find the largest plane of the bounding box.
			// The problem with this method is that this bounding box is oriented with
			// the ccoordinate system.  A better approach, implemented above, calculates
			// the average normal for the surface of interest.  That vector becomes the
			// a, b, c values for the plane of interest.  D then needs to be calculated
			// from the bounding box to ensure we are out away from the surface.


			// We need to determine the largest plane
			QList<float> sizes;

			sizes << ur[0] - ll[0];
			sizes << ur[1] - ll[1];
			sizes << ur[2] - ll[2];

			QList<float> sortedsizes = QList<float>::QList(sizes);
			qSort(sortedsizes);
			int major = sizes.indexOf(sortedsizes[2]);
			int minor = sizes.indexOf(sortedsizes[1]);
			int project = sizes.indexOf(sortedsizes[0]);

			// There might be duplicate values
			if (sortedsizes[2] == sortedsizes[1] == sortedsizes[0])
			{
				// All three will be set to 0
				// This is probably a bad situation with a good chance of this algorithm failing
				major = 0; minor = 1; project = 2;
			}
			else if (sortedsizes[2] == sortedsizes[1]) {
				// minor will be wrong in this scenario
				// This is okay as long as the major and minor axes are reasonably larger than the project axis
				if (major != 0 && project != 0)
					minor = 0;
				else if (major != 1 && project != 1)
					minor = 1;
				else if (major != 2 && project != 2)
					minor = 2;
			}
			else if (sortedsizes[1] == sortedsizes[0]) {
				// project will be wrong in this scenario
				// This is also probably a bad situation
				if (major != 0 && minor != 0)
					project = 0;
				else if (major != 1 && minor != 1)
					project = 1;
				else if (major != 2 && minor != 2)
					project = 2;
			}

			float a1[3], b1[3], c1[3];  // Three points on plane 1
			float a2[3], b2[3], c2[3];  // Three points on plane 2

			// Assign projection axis values
			a1[project] = ll[project]; b1[project] = ll[project]; c1[project] = ll[project];
			a2[project] = ur[project]; b2[project] = ur[project]; c2[project] = ur[project];

			// Assign major axis values
			a1[major] = ll[major]; b1[major] = ll[major]; c1[major] = ur[major];
			a2[major] = ur[major]; b1[major] = ur[major]; c1[major] = ll[major];

			// Assign minor axis values
			a1[minor] = ll[minor]; b1[minor] = ur[minor]; c1[minor] = ll[major];
			a2[minor] = ur[minor]; b1[minor] = ll[minor]; c1[minor] = ur[major];
			*/

			
			

		}




		// Generate the output file name
		/*QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
		if (m_GroupByPhase == true)
		{
			filename = filename + QString("Ensemble_") + QString::number(spinIter.value()) + QString("_");
		}
		filename = filename + QString("Feature_") + QString::number(grain) + ".stl";
		FILE* f = fopen(filename.toLatin1().data(), "wb");
		{
			QString ss = QObject::tr("Writing IGES for Feature Id %1").arg(grain);
			notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);
		}		
		*/
		
 


	}


	
	// Loop Over Each Surface
	// Get List of XYZ Points
	// Get List of Points on Border of Surface
	// Fit NURBS Curve to Border Points
	// Use griddata/qhull to get (u,v) surface
	// Fit surface with global interpoloation algorithm
	// Temporarily store NURBS constants
	// Write IGES File for this Grain



	/*---------------------------------------------------------------------------------------------*/
	/*--- Old Code Below Here ---------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------------*/


  // Store all the unique Spins
  
	/*
  unsigned char data[50];
  float* normal = (float*)data;
  float* vert1 = (float*)(data + 12);
  float* vert2 = (float*)(data + 24);
  float* vert3 = (float*)(data + 36);
  uint16_t* attrByteCount = (uint16_t*)(data + 48);
  *attrByteCount = 0;

  size_t totalWritten = 0;
  float u[3] = { 0.0f, 0.0f, 0.0f }, w[3] = { 0.0f, 0.0f, 0.0f };
  float length = 0.0f;

  int32_t spin = 0;
  int32_t triCount = 0;

  //Loop over the unique Spins
  for (QMap<int32_t, int32_t>::iterator spinIter = uniqueGrainIdtoPhase.begin(); spinIter != uniqueGrainIdtoPhase.end(); ++spinIter )
  {
    spin = spinIter.key();

    // Generate the output file name
    QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
    if (m_GroupByPhase == true)
    {
      filename = filename + QString("Ensemble_") + QString::number(spinIter.value()) + QString("_");
    }
    filename = filename + QString("Feature_") + QString::number(spin) + ".igs";
    FILE* f = fopen(filename.toLatin1().data(), "wb");
    {
      QString ss = QObject::tr("Writing IGES for Feature Id %1").arg(spin);
      notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);
    }

    QString header = "DREAM3D Generated For Feature ID " + QString::number(spin);
    if (m_GroupByPhase == true)
    {
      header = header + " Phase " + QString::number(spinIter.value());
    }
    err = writeHeader(f, header, 0);
    if (err < 0)
    {
    }
    triCount = 0; // Reset this to Zero. Increment for every triangle written

    // Loop over all the triangles for this spin
    for (int64_t t = 0; t < nTriangles; ++t)
    {
      // Get the true indices of the 3 nodes
      int64_t nId0 = triangles[t*3];
      int64_t nId1 = triangles[t*3+1];
      int64_t nId2 = triangles[t*3+2];

      vert1[0] = static_cast<float>(nodes[nId0*3]);
      vert1[1] = static_cast<float>(nodes[nId0*3+1]);
      vert1[2] = static_cast<float>(nodes[nId0*3+2]);

      if (m_SurfaceMeshFaceLabels[t * 2] == spin)
      {
        //winding = 0; // 0 = Write it using forward spin
      }
      else if (m_SurfaceMeshFaceLabels[t * 2 + 1] == spin)
      {
        //winding = 1; // Write it using backward spin
        // Switch the 2 node indices
        int64_t temp = nId1;
        nId1 = nId2;
        nId2 = temp;
      }
      else
      {
        continue; // We do not match either spin so move to the next triangle
      }

      vert2[0] = static_cast<float>(nodes[nId1*3]);
      vert2[1] = static_cast<float>(nodes[nId1*3+1]);
      vert2[2] = static_cast<float>(nodes[nId1*3+2]);

      vert3[0] = static_cast<float>(nodes[nId2*3]);
      vert3[1] = static_cast<float>(nodes[nId2*3+1]);
      vert3[2] = static_cast<float>(nodes[nId2*3+2]);

      // Compute the normal
      u[0] = vert2[0] - vert1[0];
      u[1] = vert2[1] - vert1[1];
      u[2] = vert2[2] - vert1[2];

      w[0] = vert3[0] - vert1[0];
      w[1] = vert3[1] - vert1[1];
      w[2] = vert3[2] - vert1[2];

      normal[0] = u[1] * w[2] - u[2] * w[1];
      normal[1] = u[2] * w[0] - u[0] * w[2];
      normal[2] = u[0] * w[1] - u[1] * w[0];

      length = sqrtf(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
      normal[0] = normal[0] / length;
      normal[1] = normal[1] / length;
      normal[2] = normal[2] / length;

      totalWritten = fwrite(data, 1, 50, f);
      if (totalWritten != 50)
      {
        QString ss = QObject::tr("Error Writing IGES File. Not enough elements written for Feature Id %1. Wrote %2 of 50.").arg(spin).arg(totalWritten);
        notifyErrorMessage(getHumanLabel(), ss, -1201);
      }
      triCount++;
    }
    fclose(f);
    err = writeNumTrianglesToFile(filename, triCount);
  }
  */
  setErrorCondition(0);
  notifyStatusMessage(getHumanLabel(), "Complete");

  return;
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t SurfaceMeshToSolidModelIges::RecurseOrderedEdges(QList<QList<int64_t>> edgelist, QList<QList<QList<int64_t>>> orderededges, QVector<bool> &visitededges, QList<QList<int64_t>> &curborder, QList<int64_t> candidates)
{

	// So, we are here because for some reason at a given edge, while searching
	// for the next edge, we found multiple points that could possibly be the 
	// next edge.  This means one of two things has happened:  1) two independent
	// loops intersect or 2) the mesh is screwed up.  Ultimately, there has to be
	// one complete loop, so, let's find the complete loop by checking each of the
	// possible loop candidates.  If we run into this situation again, then we'll 
	// have to recurse again until we finally get back to the starting point.

	// Let's set up some storage for our candidates
	QList<QList<QList<int64_t>>> candidateborderslist;
	QList<QVector<bool>> candidatevisitedlist;
	QList<int32_t> candidatestatuslist;

	// Let's loop over all possible candidates
	for (QList<int64_t>::iterator candidate = candidates.begin(); candidate != candidates.end(); ++candidate)
	{
		
		// Let's make our own copy of curborder and visitededges
		QList<QList<int64_t>> candidateborder = QList<QList<int64_t>>(curborder);
		QVector<bool> candidatevisited = QVector<bool>(visitededges);

		// Picking up from where we were - add this candidate to the border
		QList<int64_t> tmplist;
		if (edgelist[*candidate][1] == curborder.last()[1])
		{
			tmplist << edgelist[*candidate][1];
			tmplist << edgelist[*candidate][0];
		}
		else
			tmplist = edgelist[*candidate];

		candidateborder << tmplist;

		// Let's make sure we don't get stuck because of bad points
		// in the mesh and mark all candidates as visited
		// We will undo this later to ensure that intersecting loops
		// are found if this candidate completes the loop.
		for (QList<int64_t>::iterator k = candidates.begin(); k != candidates.end(); ++k)
		{
			candidatevisited[*k] = true;
		}

		// Error tracking
		int32_t errorstate = 0;
		
		// Now we continue as we were before
		while (candidateborder.last()[1] != candidateborder.first()[0]){

			// Candidate edges
			QList<int64_t> newcandidates;
			bool reversed = false;

			// Let's loop through the list and find possible next points
			for (int64_t i = 0; i < edgelist.size(); ++i)
			{
				if (candidatevisited[i] == true)
					continue;  // Skip this point - we've been here already

				if (edgelist[i][0] == candidateborder.last()[1])
					newcandidates << i;  // Trailing value that match the current leading value
				else if (edgelist[i][1] == candidateborder.last()[1])
				{
					newcandidates << i;
					reversed = true;
				}
			}

			if (newcandidates.size() == 1) // We've found exactly one match
			{
				QList<int64_t> tmplist;
				if (reversed == true)
				{
					tmplist << edgelist[newcandidates[0]][1];
					tmplist << edgelist[newcandidates[0]][0];
				}
				else
					tmplist = edgelist[newcandidates[0]];

				candidateborder << tmplist;
				candidatevisited[newcandidates[0]] = true;
			}
			else if (newcandidates.size() > 1) // We've found multiple matches
			{
				// Looks like we found another loop
				errorstate = RecurseOrderedEdges(edgelist, orderededges, candidatevisited, candidateborder, newcandidates);
				if (errorstate < 0)
					break;
			}
			else
			{
				// Looks like we found a bad mesh point
				// Let's break out and mark this candidate as bad
				errorstate = -1;
				break;
			}


		}

		candidateborderslist << candidateborder;
		candidatevisitedlist << candidatevisited;
		candidatestatuslist << errorstate;

	}
	
	// We have now looped through all candidates
	// Let's see how many good candidates we found
	int32_t goodcandidatecount = 0;
	int32_t goodcandidateid = 0;
	for (int32_t i = 0; i < candidatestatuslist.size(); ++i)
	{
		if (candidatestatuslist[i] >= 0)
		{
			++goodcandidatecount;
			goodcandidateid = i;
		}
			
	}

	// Let's handle the various cases
	if (goodcandidatecount == 0)
	{
		// We didn't find any good candidates - return an error
		return -1;
	}
	else if (goodcandidatecount > 1)
	{
		// We found multiple ways to complete the loop - not good
		// Let's take the path that encompases the most points
		int32_t maxsize = 0;
		for (int32_t i = 0; i < candidateborderslist.size(); ++i)
		{
			if (candidateborderslist[i].size() > maxsize)
			{
				maxsize = candidateborderslist[i].size();
				goodcandidateid = i;
			}
		}
	}

	// Let's update the border for our caller to save effort
	curborder = QList<QList<int64_t>>(candidateborderslist[goodcandidateid]);
	visitededges = QVector<bool>(candidatevisitedlist[goodcandidateid]);

	// Let's re-mark the other candidates as being okay - they probably
    // aren't, but if this was a case of two independent loops intersecting
	// then we need the edge for the other loop still

	for (QList<int64_t>::iterator k = candidates.begin(); k != candidates.end(); ++k)
	{
		if (*k != candidates[goodcandidateid])
			visitededges[*k] = false;
	}

	// return good status
	return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QList<int32_t> SurfaceMeshToSolidModelIges::RecurseTrianglesOnSurface(ElementDynamicList::Pointer m_TriangleNeighbors, QVector<bool> &checkedtriangles, int64_t t, int32_t grain, int32_t featurefacelabel)
{
	// Recursion here won't actually work... we'll overflow the stack.
	// Instead, we need to maintain a list of triangles to check and
	// keep looping until we've exhausted the list 

	QList<int32_t> returnlist;	

	// Iterator List
	QList<int64_t> iteratorlist;
	iteratorlist << t;

	// Loop list
	while (iteratorlist.size() > 0)
	{
		// Let's start with the first point
		returnlist << iteratorlist[0];
		//checkedtriangles[iteratorlist[0]] = true;

		// Let's get this triangles neighbors
		uint16_t neighborcount = m_TriangleNeighbors->getNumberOfElements(iteratorlist[0]);
		int64_t* nList = m_TriangleNeighbors->getElementListPointer(iteratorlist[0]);

		// Loop over each neighbor
		for (int64_t i = 0; i < neighborcount; ++i)
		{
			// Is this neighbor on the same surface and not already on the list
			if (checkedtriangles[nList[i]] == false && (m_SurfaceMeshFeatureFaceIds[nList[i]] == featurefacelabel && (m_SurfaceMeshFaceLabels[2 * nList[i]] == grain || m_SurfaceMeshFaceLabels[2 * nList[i] + 1] == grain)))
			{
				iteratorlist << nList[i];
			}
			checkedtriangles[nList[i]] = true;
		}

		// Remove this item from the list
		iteratorlist.removeFirst();
	}

	return returnlist;
	
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t SurfaceMeshToSolidModelIges::writeHeader(FILE* f, const QString& header, int32_t triCount)
{
  if (NULL == f)
  {
    return -1;
  }
  char h[80];
  size_t headlength = 80;
  if (header.length() < 80) { headlength = header.length(); }
  ::memset(h, 0, 80);
  ::memcpy(h, header.data(), headlength);
  // Return the number of bytes written - which should be 80
  fwrite(h, 1, 80, f);
  fwrite(&triCount, 1, 4, f);
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t SurfaceMeshToSolidModelIges::writeNumTrianglesToFile(const QString& filename, int32_t triCount)
{
  // We need to update the number of triangles in the file
  int32_t err = 0;

  FILE* out = fopen(filename.toLatin1().data(), "r+b");
  fseek(out, 80L, SEEK_SET);
  fwrite( (char*)(&triCount), 1, 4, out);
  fclose(out);

  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer SurfaceMeshToSolidModelIges::newFilterInstance(bool copyFilterParameters)
{
	SurfaceMeshToSolidModelIges::Pointer filter = SurfaceMeshToSolidModelIges::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SurfaceMeshToSolidModelIges::getCompiledLibraryName()
{ return IOConstants::IOBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SurfaceMeshToSolidModelIges::getGroupName()
{ return DREAM3D::FilterGroups::IOFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SurfaceMeshToSolidModelIges::getSubGroupName()
{ return DREAM3D::FilterSubGroups::OutputFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SurfaceMeshToSolidModelIges::getHumanLabel()
{ return "Write IGES Solid Model Files from Surface Mesh"; }
