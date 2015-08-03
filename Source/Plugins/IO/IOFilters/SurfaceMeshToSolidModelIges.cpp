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
#include <ctime>

// Structure
struct surfdata {
	double *U;
	double *V;
	double *P;
	double *UE;
	double *PE;
	int64_t xcnt;
	int64_t ycnt;
	int64_t edgecnt;
};

inline double max(double a, double b, double c)
{
	if (a < b)
		return (b < c ? c : b);
	else
		return (a < c ? c : a);
}

inline double min(double a, double b, double c)
{
	if (a > b)
		return (b > c ? c : b);
	else
		return (a > c ? c : a);
}

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
		QList<QList<int32_t> > internalSurfaceTriangles;	// List of internal surface triangles for each found surface
		QList<QList<int32_t> > tmpExternalSurfaceTriangles;	// List of external surface triangles for each found surface
		QList<QList<int32_t> > externalSurfaceTriangles;	// List of external surface triangles for each found surface

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
							tmpExternalSurfaceTriangles[i] << t;
							// Break out of the loop
							found = true;
							break;
						}
					}

					// We didn't find a triangle with the same normal - so let's add this to our list
					if (!found)
					{
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t];
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t + 1];
						uniqueExternalSurfaceNormals << m_SurfaceMeshTriangleNormals[3 * t + 2];
						tmpExternalSurfaceTriangles << QList<int32_t>();
						tmpExternalSurfaceTriangles[uniqueExternalSurfacesCount] << t;
						uniqueExternalSurfacesCount++;
					}

				}
			}
		}

		// We've made a first pass and collected a list of unique normals
		// Now let's go back through again and make sure these surfaces are continuous
		if (externalSurfaceFaceLabel > -1)
		{
			
			// Now let's find all of the triangles with those feature faces and grain ids so we can iterate them later
			for (int32_t facecnt = 0; facecnt < uniqueExternalSurfacesCount; ++facecnt)
			{
				// Set up storage
				QVector<bool> checkedtriangles(nTriangles, false);
				int64_t surfcnt = 0;
				// Let's look for matching triangles
				for (int64_t t = 0; t < nTriangles; ++t)
				{
					// Have we found a triangle on this grain and surface
					if (checkedtriangles[t] == false && m_SurfaceMeshFeatureFaceIds[t] == externalSurfaceFaceLabel && (m_SurfaceMeshFaceLabels[2*t] == grain || m_SurfaceMeshFaceLabels[2*t+1] == grain) && 
						m_SurfaceMeshTriangleNormals[3*t] == uniqueExternalSurfaceNormals[3*facecnt] && 
						m_SurfaceMeshTriangleNormals[3*t+1] == uniqueExternalSurfaceNormals[3*facecnt+1] &&
						m_SurfaceMeshTriangleNormals[3*t+2] == uniqueExternalSurfaceNormals[3*facecnt+2])
					{
						// Let's recurse until we find all of the adjoining triangles
						externalSurfaceTriangles << RecurseExternalTrianglesOnSurface(m_TriangleNeighbors, checkedtriangles, t, grain, externalSurfaceFaceLabel, uniqueExternalSurfaceNormals[3*facecnt], uniqueExternalSurfaceNormals[3*facecnt+1], uniqueExternalSurfaceNormals[3*facecnt+2]);
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
						uniqueExternalSurfaceNormals.insert(3*facecnt+3, uniqueExternalSurfaceNormals[3*facecnt+2]);
						uniqueExternalSurfaceNormals.insert(3*facecnt+3, uniqueExternalSurfaceNormals[3*facecnt+1]);
						uniqueExternalSurfaceNormals.insert(3*facecnt+3, uniqueExternalSurfaceNormals[3*facecnt+0]);
						++uniqueExternalSurfacesCount;
						++facecnt;
					}
				}
			}			
		}

		QList<surfdata> interpsurfaces;


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
				surfnorm[0] += m_SurfaceMeshTriangleNormals[3 * (*t) + 0];
				surfnorm[1] += m_SurfaceMeshTriangleNormals[3 * (*t) + 1];
				surfnorm[2] += m_SurfaceMeshTriangleNormals[3 * (*t) + 2];
				/*float a[3], b[3], c[3], n[3];
				triangleGeom->getVertCoordsAtTri(*t, a, b, c);
				GeometryMath::FindPlaneNormalVector(a, b, c, n);
				surfnorm[0] += n[0];
				surfnorm[1] += n[1];
				surfnorm[2] += n[2];*/
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


			double ll[3], ur[3];
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

			// We now have a bounding box in which the "X-Y Plane" described by the
			// point sets 1,4,6,7 and 2,3,5,8 are the planes we need to build a grid
			// across.  Let's find the dimensions and use linspace and the coordinate
			// transform to get two sets of coordinates for ray tracing

			double xdim = ur[0] - ll[0];
			double ydim = ur[1] - ll[1];

			int32_t xcnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * xdim / ydim)));
			int32_t ycnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * ydim / xdim)));

			double *griddedsurf = new double[xcnt*ycnt * 3];

			std::vector<double> xvals = DREAM3DMath::linspace(ll[0], ur[0], xcnt);
			std::vector<double> yvals = DREAM3DMath::linspace(ll[1], ur[1], ycnt);

			// Rotation may still be needed into a better coordinate frame
			// to help deal with situations in which the plane is vertical

			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					griddedsurf[0 + j * 3 + i * ycnt * 3] = xvals[i];
					griddedsurf[1 + j * 3 + i * ycnt * 3] = yvals[j];
				}
			}

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
				assert(false);

			// Let's build a unique list of points for this surface
			QMap<int32_t, int32_t> pointindexmap;
			for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
			{
				pointindexmap.insert(triangles[3 * (*t) + 0], 0);
				pointindexmap.insert(triangles[3 * (*t) + 1], 0);
				pointindexmap.insert(triangles[3 * (*t) + 2], 0);
			}

			double *pt_array = new double[2 * pointindexmap.size()];
			double *pt_zarray = new double[pointindexmap.size()];

			QMap<int32_t, int32_t>::iterator curpt = pointindexmap.begin();			
			for (int64_t i = 0; i < pointindexmap.size(); ++i)
			{
				float precoords[3];
				float coords[3];
				triangleGeom->getCoords(curpt.key(), precoords);
				MatrixMath::Multiply3x3with3x1(rotmat, precoords, coords);
				pt_array[2 * i + 0] = (double)coords[0];
				pt_array[2 * i + 1] = (double)coords[1];
				pt_zarray[i] = (double)coords[2];
				++curpt;
			}

			QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain") + QString::number(grain) + QString("_Surface") + QString::number(surfcnt) + QString("_RotatedRawPoints_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			FILE *f = fopen(filename.toLatin1().data(), "w");
			for (int64_t i = 0; i < pointindexmap.size(); ++i)
			{
				char line[100];
				int cnt = sprintf(line, "%f,%f,%f\n", pt_array[2 * i + 0], pt_array[2 * i + 1], pt_zarray[i]);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);

			//*********************************************************
			//*** Start Qhull Code Below Here                       ***
			//*********************************************************
			// This code is functionally equivalent to using Octave's
			// griddata function.  This code is actually adapted from
			// __delaunayn__.cc, delaunayn.m, delaunay.m, griddata.m
			// and tsearch.cc in Octave.  So, accordingly, appropriate
			// copyright credit is given to the folks at Octave.
			// Additional credit goes to the folks that developed the
			// Quick Hull library as well.
			//*********************************************************

			// This code adapted from __delaunayn__.cc in Octave
			// This is the wrapper code around qhull that actually
			// performs the Delaunay triangulation process.  The
			// input is the set of x,y values (z values are ignored
			// for the time being) that make up the vertex points
			// from the mesh.  The output is a list of triangle
			// vertices in barycentric coordinates.

			boolT ismalloc = false;
			const int dim = 2;
			const int n = pointindexmap.size();
			char *options = "qhull d Qt Qbb Qc Qz";  // Consider adding Qj - probably not needed - will come with a likely performance hit

			#if defined (CMP_HAVE_WINDOWS)
				FILE *outfile = fopen("NUL", "w");
			#else
				FILE *outfile = fopen("/dev/null", "w");
			#endif

			FILE *errfile = stderr;

			int exitcode = qh_new_qhull(dim, n, pt_array, ismalloc, options, outfile, errfile);

			facetT *facet;
			vertexT *vertex, **vertexp;
			int64_t nf = 0;
			int64_t curcnt = 0;

			if (!exitcode)
			{
				qh_triangulate();

				FORALLfacets
				{
					if (! facet->upperdelaunay)
						nf++;

					if (! facet->simplicial)
					{
						assert(false);  // Need actual error handling here
						exitcode = 1;
						break;
					}
				}
			}

			int *tri = new int[3 * nf];

			if (!exitcode)
			{
				FORALLfacets
				{
					if (!facet->upperdelaunay)
					{
						int64_t j = 0;
						FOREACHvertex_(facet->vertices)
						{
							tri[3 * curcnt + (j++)] = qh_pointid(vertex->point);
						}
						curcnt++;
					}
				}

			}
			else
				assert(false); // Fix Me

			qh_freeqhull(!qh_ALL);

			int curlong, totlong;
			qh_memfreeshort(&curlong, &totlong);

			if (curlong || totlong)
				assert(false); // Fix Me


			fclose(outfile);

			// The code below is adapted from delaunayn.m in Octave.
			// This cleans up the results from a Delaunay triangulation
			// by removing the zero volume simplices.

			QList<int64_t> goodids;

			for (int64_t i = 0; i < nf; ++i)
			{
				float X[2][2];
				float det;
				float sumsq[2];
				X[0][0] = pt_array[2 * tri[3 * i + 0] + 0] - pt_array[2 * tri[3 * i + 1] + 0];
				X[0][1] = pt_array[2 * tri[3 * i + 0] + 1] - pt_array[2 * tri[3 * i + 1] + 1];
				X[1][0] = pt_array[2 * tri[3 * i + 1] + 0] - pt_array[2 * tri[3 * i + 2] + 0];
				X[1][1] = pt_array[2 * tri[3 * i + 1] + 1] - pt_array[2 * tri[3 * i + 2] + 1];
				det = abs(X[0][0] * X[1][1] - X[0][1] * X[1][0]);
				sumsq[0] = (X[0][0] * X[0][0]) + (X[0][1] * X[0][1]);
				sumsq[1] = (X[1][0] * X[1][0]) + (X[1][1] * X[1][1]);
				if ((det / sqrt(sumsq[0]) < (1e3 * DBL_EPSILON)) && (det / sqrt(sumsq[1]) < (1e3 * DBL_EPSILON)))
					;
				else
					goodids << i;
			}

			int *newtri = new int[3 * goodids.size()];
			
			
			for (int64_t i = 0; i < goodids.size(); ++i)
			{
				newtri[3 * i + 0] = tri[3 * goodids[i] + 0];
				newtri[3 * i + 1] = tri[3 * goodids[i] + 1];
				newtri[3 * i + 2] = tri[3 * goodids[i] + 2];
			}

			delete[] tri;

			// This code is adapted from tsearch.cc in Octave
			// Here we are searching for the triangles associated
			// with each point in the interpolation grid.
			int64_t nelem = goodids.size();

			// We need the bounds for each simplex
			double *minx = new double[nelem];
			double *maxx = new double[nelem];
			double *miny = new double[nelem];
			double *maxy = new double[nelem];

			for (int64_t i = 0; i < nelem; ++i)
			{
				minx[i] = min(pt_array[2 * newtri[3 * i + 0] + 0], pt_array[2 * newtri[3 * i + 1] + 0], pt_array[2 * newtri[3 * i + 2] + 0]) - DBL_EPSILON;
				maxx[i] = max(pt_array[2 * newtri[3 * i + 0] + 0], pt_array[2 * newtri[3 * i + 1] + 0], pt_array[2 * newtri[3 * i + 2] + 0]) + DBL_EPSILON;
				miny[i] = min(pt_array[2 * newtri[3 * i + 0] + 1], pt_array[2 * newtri[3 * i + 1] + 1], pt_array[2 * newtri[3 * i + 2] + 1]) - DBL_EPSILON;
				maxy[i] = max(pt_array[2 * newtri[3 * i + 0] + 1], pt_array[2 * newtri[3 * i + 1] + 1], pt_array[2 * newtri[3 * i + 2] + 1]) + DBL_EPSILON;
			}

			// We are going to combine tsearch.cc and griddata.m into one pass
			
			// So, let's loop over every element in the output grid and find
			// the triangulated point.
			double x0, y0, a11, a12, a21, a22, det;
			x0 = y0 = 0.0;
			a11 = a12 = a21 = a22 = 0.0;
			det = 0.0;

			int64_t k = nelem; // Triangle counter

			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					// Current Values for Convenience
					const double xt = griddedsurf[0 + 3 * j + i * ycnt * 3];
					const double yt = griddedsurf[1 + 3 * j + i * ycnt * 3];

					int64_t foundelem = -1;

					// check if last triangle contains the next point
					// This will potentially substantially cut down the
					// number of complete loop iterations needed
					if (k < nelem)
					{
						const double dx1 = xt - x0;
						const double dx2 = yt - y0;
						const double c1 = (a22 * dx1 - a21 * dx2) / det;
						const double c2 = (-a12 * dx1 + a11 * dx2) / det;
						if (c1 >= -DBL_EPSILON && c2 >= -DBL_EPSILON && (c1 + c2) <= (1 + DBL_EPSILON))
						{
							foundelem = k;
						}
					}
					if (foundelem < 0)
					{
						for (k = 0; k < nelem; k++)
						{
							// Potentially a good place to cancel

							if (xt >= minx[k] && xt <= maxx[k] && yt >= miny[k] && yt <= maxy[k])
							{
								// element inside the minimum rectangle: examine it closely
								x0 = pt_array[2 * newtri[3 * k + 0] + 0];
								y0 = pt_array[2 * newtri[3 * k + 0] + 1];
								a11 = pt_array[2 * newtri[3 * k + 1] + 0] - x0;
								a12 = pt_array[2 * newtri[3 * k + 1] + 1] - y0;
								a21 = pt_array[2 * newtri[3 * k + 2] + 0] - x0;
								a22 = pt_array[2 * newtri[3 * k + 2] + 1] - y0;
								det = a11 * a22 - a21 * a12;

								// solve the system
								const double dx1 = xt - x0;
								const double dx2 = yt - y0;
								const double c1 = (a22 * dx1 - a21 * dx2) / det;
								const double c2 = (-a12 * dx1 + a11 * dx2) / det;
								if ((c1 >= -DBL_EPSILON) && (c2 >= -DBL_EPSILON) && ((c1 + c2) <= (1 + DBL_EPSILON)))
								{
									foundelem = k;
									break;
								}
							} //endif # examine this element closely
						} //endfor # each element
					}

					if (foundelem < 0)
					{
						// We didn't find the point - so let's set nan for now
						griddedsurf[2 + 3 * j + i * ycnt * 3] = nan("");
					}
					else
					{
						// We found the triangle - let's calculate the point
						float a[3], b[3], c[3], n[3], d;

						a[0] = pt_array[2 * newtri[3 * foundelem + 0] + 0];
						a[1] = pt_array[2 * newtri[3 * foundelem + 0] + 1];
						a[2] = pt_zarray[newtri[3 * foundelem + 0]];

						b[0] = pt_array[2 * newtri[3 * foundelem + 1] + 0];
						b[1] = pt_array[2 * newtri[3 * foundelem + 1] + 1];
						b[2] = pt_zarray[newtri[3 * foundelem + 1]];

						c[0] = pt_array[2 * newtri[3 * foundelem + 2] + 0];
						c[1] = pt_array[2 * newtri[3 * foundelem + 2] + 1];
						c[2] = pt_zarray[newtri[3 * foundelem + 2]];

						GeometryMath::FindPlaneNormalVector(a, b, c, n);
						MatrixMath::Normalize3x1(n);
						
						d = -(a[0]*n[0] + a[1]*n[1] + a[2]*n[2]);

						griddedsurf[2 + 3 * j + i * ycnt * 3] = -(n[0]*xt + n[1]*yt + d) / n[2];
					}
				}
			}


			struct neighbor {
				int64_t x;
				int64_t y;
				int64_t z;
				float d;
			};


			// Let's extrapolate the edges and unfilled points
			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{

					if (isnan(griddedsurf[2 + 3*j + i * ycnt * 3]))
					{
						// At each point, we need to find the nearest neighbors and perform
						// a weighted average based on distance.  We will find the neighbors
						// by dividing up the grid into quadrants.  We will progress radially
						// outward in each quadrant until we find a radius that results in
						// us finding the first neighbor in that quadrant.  We will take all
						// neighbors on that radius in that quadrant.  We repeat this process
						// for each quadrant.  Each quadrant will include the first axes from
						// the standard perspective.  The second axes will be included with the
						// subsequent quadrant.						

						int64_t x=i, y=j;
						int64_t curradius = 1;						
						QList<neighbor> curpointneighbors;
						int64_t curncount = 0;
						int64_t curmaxradius = (xcnt-1-i) > (ycnt-1-j) ? (xcnt-1-i) : (ycnt-1-j);
	
						// First quadrant
						while (true)
						{

							if (x<i+curradius && y==j)
								++x;
							else if (x==i+curradius && y < j + curradius)
								++y;
							else if (x>i && y==j+curradius)
								--x;
							else
								if (curncount > 0 || curradius >= curmaxradius)
									break;
								else
									{ ++curradius; x = i; y = j; continue; }
							
							if (x >= xcnt || x < 0 || y >= ycnt || y < 0)
								continue; 
							
							if (!isnan(griddedsurf[2 + 3*y + x*ycnt*3 ]))
							{
								neighbor n;
								n.x = x;
								n.y = y;
								n.z = griddedsurf[2 + 3*y + x*ycnt*3 ];
								n.d = sqrt(abs(x-i) * abs(x-i) + abs(y-j) * abs(y-j));
								curpointneighbors << n;
								++curncount;
							}
						}



						x=i; y=j;
						curradius = 1;						
						curncount = 0;
						curmaxradius = (i) > (ycnt-1-j) ? (i) : (ycnt-1-j);
	
						// Second quadrant
						while (true)
						{

							if (x==i && y < j + curradius)
								++y;
							else if (x>i-curradius && y==j+curradius)
								--x;
							else if (x==i-curradius && y > j)
								--y;
							else
								if (curncount > 0 || curradius >= curmaxradius)
									break;
								else
									{ ++curradius; x = i; y = j; continue; }
							
							if (x >= xcnt || x < 0 || y >= ycnt || y < 0)
								continue; 
							
							if (!isnan(griddedsurf[2 + 3*y + x*ycnt*3 ]))
							{
								neighbor n;
								n.x = x;
								n.y = y;
								n.z = griddedsurf[2 + 3*y + x*ycnt*3 ];
								n.d = sqrt(abs(x-i) * abs(x-i) + abs(y-j) * abs(y-j));
								curpointneighbors << n;
								++curncount;
							}
						}

						x=i; y=j;
						curradius = 1;						
						curncount = 0;
						curmaxradius = (i) > (j) ? (i) : (j);
	
						// Third quadrant
						while (true)
						{

							if (x > i - curradius && y == j)
								--x;
							else if (x== i-curradius && y > j-curradius)
								--y;
							else if (x < i && y == j-curradius)
								++x;
							else
								if (curncount > 0 || curradius >= curmaxradius)
									break;
								else
									{ ++curradius; x = i; y = j; continue; }
							
							if (x >= xcnt || x < 0 || y >= ycnt || y < 0)
								continue; 
							
							if (!isnan(griddedsurf[2 + 3*y + x*ycnt*3 ]))
							{
								neighbor n;
								n.x = x;
								n.y = y;
								n.z = griddedsurf[2 + 3*y + x*ycnt*3 ];
								n.d = sqrt(abs(x-i) * abs(x-i) + abs(y-j) * abs(y-j));
								curpointneighbors << n;
								++curncount;
							}
						}

						x=i; y=j;
						curradius = 1;						
						curncount = 0;
						curmaxradius = (xcnt - 1 - i) > (j) ? (xcnt - 1 - i) : (j);
	
						// Fourth quadrant
						while (true)
						{

							if (x == i && y > j - curradius)
								--y;
							else if (x < i+curradius && y == j - curradius )
								++x;
							else if (x == i+curradius && y < j)
								++y;
							else
								if (curncount > 0 || curradius >= curmaxradius)
									break;
								else
									{ ++curradius; x = i; y = j; continue; }
							
							if (x >= xcnt || x < 0 || y >= ycnt || y < 0)
								continue; 
							
							if (!isnan(griddedsurf[2 + 3*y + x*ycnt*3 ]))
							{
								neighbor n;
								n.x = x;
								n.y = y;
								n.z = griddedsurf[2 + 3*y + x*ycnt*3 ];
								n.d = sqrt(abs(x-i) * abs(x-i) + abs(y-j) * abs(y-j));
								curpointneighbors << n;
								++curncount;
							}
						}


						float totalweight = 0.0f;
						for (int64_t q = 0; q < curpointneighbors.size(); ++q)
						{
							totalweight += 1 / (curpointneighbors[q].d * curpointneighbors[q].d);
						}

						float interpvalue = 0.0f;
						for (int64_t q = 0; q < curpointneighbors.size(); ++q)
						{
							interpvalue += curpointneighbors[q].z * ((1 / (curpointneighbors[q].d * curpointneighbors[q].d)) / totalweight);
						}


						griddedsurf[2 + 3 * j + i * ycnt * 3] = interpvalue;
						
					}
				}
			}








			// Free memory on the heap
			delete[] minx;
			delete[] miny;
			delete[] maxx;
			delete[] maxy;
			delete[] newtri;
			delete[] pt_array;
			delete[] pt_zarray;

			// Rotate back
			MatrixMath::TransformationMatrixFromAToB(surfnorm, tovec, rotmat);
			double dblrotmat[3][3];
			dblrotmat[0][0] = (double)rotmat[0][0];
			dblrotmat[0][1] = (double)rotmat[0][1];
			dblrotmat[0][2] = (double)rotmat[0][2];
			dblrotmat[1][0] = (double)rotmat[1][0];
			dblrotmat[1][1] = (double)rotmat[1][1];
			dblrotmat[1][2] = (double)rotmat[1][2];
			dblrotmat[2][0] = (double)rotmat[2][0];
			dblrotmat[2][1] = (double)rotmat[2][1];
			dblrotmat[2][2] = (double)rotmat[2][2];
			double invmat[3][3];
			MatrixMath::Invert3x3(dblrotmat, invmat);
			MatrixMath::Transpose3x3(invmat, dblrotmat);
			
			for (int64_t i = 0; i < xcnt; ++i)
			{
				for (int64_t j = 0; j < ycnt; ++j)
				{
					double prerot[3], postrot[3];
					prerot[0] = griddedsurf[0 + 3 * j + i * ycnt * 3];
					prerot[1] = griddedsurf[1 + 3 * j + i * ycnt * 3];
					prerot[2] = griddedsurf[2 + 3 * j + i * ycnt * 3];
					MatrixMath::Multiply3x3with3x1(dblrotmat, prerot, postrot);
					griddedsurf[0 + 3 * j + i * ycnt * 3] = postrot[0];
					griddedsurf[1 + 3 * j + i * ycnt * 3] = postrot[1];
					griddedsurf[2 + 3 * j + i * ycnt * 3] = postrot[2];
				}
			}


			// Get Edge List Ready
			double *A = new double[orderededges[0].size() * 3];
			for (QList<QList<int64_t>>::iterator i = orderededges[0].begin(); i != orderededges[0].end(); ++i)
			{
				float a = static_cast<float>(nodes[(*i)[0] * 3]);
				float b = static_cast<float>(nodes[(*i)[0] * 3 + 1]);
				float c = static_cast<float>(nodes[(*i)[0] * 3 + 2]);
				int64_t cnt = i - orderededges[0].begin();
				A[3 * (cnt) + 0] = a;
				A[3 * (cnt) + 1] = b;
				A[3 * (cnt) + 2] = c;
			}

			double *U = new double[xcnt+3];
			double *V = new double[ycnt+3];
			double *P = new double[xcnt*ycnt*3];
			double *UE = new double[orderededges[0].size() + 1];
			double *PE = new double[orderededges[0].size() * 3];
			int64_t m;

			GlobalSurfInterp(xcnt - 1, ycnt - 1, griddedsurf, 3, 3, U, V, P);
			GlobalCurveInterp(orderededges[0].size() - 1, A, 3, 1, m, UE, PE);

			
			surfdata thissurf;
			thissurf.P = P;
			thissurf.U = U;
			thissurf.V = V;
			thissurf.xcnt = xcnt;
			thissurf.ycnt = ycnt;
			thissurf.PE = PE;
			thissurf.UE = UE;
			thissurf.edgecnt = orderededges[0].size();

			interpsurfaces << thissurf;		

			// Temporary surface output
			/*filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain") + QString::number(grain) + QString("_Surface") + QString::number(surfcnt) + QString("_Gridded_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
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
			filename = filename + QString("Grain") + QString::number(grain) + QString("_Surface") + QString::number(surfcnt) + QString("_RawPoints_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
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

			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain") + QString::number(grain) + QString("_Surface") + QString::number(surfcnt) + QString("_Edges_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			f = fopen(filename.toLatin1().data(), "w");
			for (QList<QList<int64_t>>::iterator i = orderededges[0].begin(); i != orderededges[0].end(); ++i)
			{
				float a = static_cast<float>(nodes[(*i)[0] * 3]);
				float b = static_cast<float>(nodes[(*i)[0] * 3 + 1]);
				float c = static_cast<float>(nodes[(*i)[0] * 3 + 2]);
				char line[100];
				int cnt = sprintf(line, "%f,%f,%f\n", a, b, c);
				fwrite(line, sizeof(char), cnt, f);
			}
			fclose(f);

			*/


			delete[] griddedsurf;		

		}

		// We are now ready to write out this file
		QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
		filename = filename + QString("_Grain") + QString::number(grain) + ".igs";
		FILE *f = fopen(filename.toLatin1().data(), "w");
		char line[100];
		int64_t cnt;

		cnt = sprintf(line, "DREAM3D Surface Mesh to Solid Model IGES File                           S0000001\n");
		fwrite(line, sizeof(char), cnt, f);
		
		QString paramsect = "";

		paramsect += "1H,,";										// Parameter Separator
		paramsect += "1H;,";										// Record Separator
		paramsect += "7HDREAM3D,";									// Product Identifier

		// Filename
		paramsect.sprintf("%s%dH%s,", paramsect.toLatin1().data(), filename.size(), filename.toLatin1().data());

		paramsect += "7HDREAM3D,";									// System Identifier
		paramsect += "7HDREAM3D,";									// Preprocessor Identifier
		paramsect += "64,";											// Integer Size
		paramsect += "38,";											// Single Float Size
		paramsect += "6,";											// Single Precision
		paramsect += "308,";										// Double Float Size
		paramsect += "15,";											// Double Precision
		paramsect += "7HDREAM3D,";									// Receiving Identifier
		paramsect += "1.,";											// Scale Factor
		paramsect += "9,";											// Unit flag for um
		paramsect += "2HUM,";										// Unit name
		paramsect += "1,";											// Max Num Line Weight Graduations
		paramsect += "0.125,";										// Width of Max Line

		// Timestamp
		time_t t = time(0);   // get time now
		struct tm * now = localtime(&t);
		paramsect += "15H" + QString::number(now->tm_year + 1900)
			+ ((now->tm_mon + 1 < 10) ? ("0" + QString::number(now->tm_mon + 1)) : (QString::number(now->tm_mon + 1)))
			+ ((now->tm_mday < 10) ? ("0" + QString::number(now->tm_mon)) : (QString::number(now->tm_mon)))
			+ "." + ((now->tm_hour + 1 < 10) ? ("0" + QString::number(now->tm_hour + 1)) : (QString::number(now->tm_hour + 1)))
			+ ((now->tm_min + 1 < 10) ? ("0" + QString::number(now->tm_min + 1)) : (QString::number(now->tm_min + 1)))
			+ ((now->tm_sec + 1 < 10) ? ("0" + QString::number(now->tm_sec + 1)) : (QString::number(now->tm_sec + 1)))
			+ ",";

		paramsect += "1E-008,";										// Tolerance
		paramsect += "20000,";										// Max Value
		paramsect += "7HDREAM3D,";									// Author
		paramsect += "7HDREAM3D,";									// Org
		paramsect += "10,";											// Version 5.2
		paramsect += "0,";											// No Drafting Standard

		// Timestamp
		paramsect += "15H" + QString::number(now->tm_year + 1900)
			+ ((now->tm_mon + 1 < 10) ? ("0" + QString::number(now->tm_mon + 1)) : (QString::number(now->tm_mon + 1)))
			+ ((now->tm_mday < 10) ? ("0" + QString::number(now->tm_mon)) : (QString::number(now->tm_mon)))
			+ "." + ((now->tm_hour + 1 < 10) ? ("0" + QString::number(now->tm_hour + 1)) : (QString::number(now->tm_hour + 1)))
			+ ((now->tm_min + 1 < 10) ? ("0" + QString::number(now->tm_min + 1)) : (QString::number(now->tm_min + 1)))
			+ ((now->tm_sec + 1 < 10) ? ("0" + QString::number(now->tm_sec + 1)) : (QString::number(now->tm_sec + 1)))
			+ ";";

		//char *paramstr = paramsect.toLatin1().data();
		char padstr[8] = "0000000";
		int64_t i = 0;
		int64_t curcnt = 1;
		while (i < paramsect.size())
		{
			for (int64_t curline = 0; curline < 72; ++curline)
			{
				if (i < paramsect.size())
				{
					char val = paramsect.at(i).toLatin1();
					fwrite(&val, sizeof(char), 1, f);
					++i;
				}
				else
				{
					fwrite(" ", sizeof(char), 1, f);
				}
			}
			fwrite("G", sizeof(char), 1, f);
			//for (int64_t j = 0; j < 7; j++)	padstr[j] = "0";
			QString curval = QString::number(curcnt);
			int64_t thissize = curval.size();
			for (int64_t j = 0; j < thissize; ++j)
			{
				char val = curval.at(j).toLatin1();
				padstr[7 - thissize + j] = val;
			}
			++curcnt;
			fwrite(padstr, sizeof(char), 7, f);
			fwrite("\n", sizeof(char), 1, f);
		}

		QList<int64_t> dirlinecnt;
		QList<int64_t> paramlinecnt;

		int64_t curdirline = 1;
		int64_t curparamline = 1;

		int64_t curcharcnt = 0;
		char buf[1000];
		int64_t curcnt = 0;

		for (QList<surfdata>::iterator cursurf = interpsurfaces.begin(); cursurf != interpsurfaces.end(); ++cursurf)
		{
			// 128
			dirlinecnt << curdirline;
			curdirline += 2;
			curcharcnt = 4;
			curcharcnt += sprintf(buf, "%d,", (*cursurf).xcnt);
			curcharcnt += sprintf(buf, "%d,", (*cursurf).ycnt);


			// 126
			dirlinecnt << curdirline;
			curdirline += 2;

			// 141
			dirlinecnt << curdirline;
			curdirline += 2;

			// 143
			dirlinecnt << curdirline;
			curdirline += 2;


		}

		fclose(f);
		
	}
	
	


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

QString SurfaceMeshToSolidModelIges::Build128(surfdata surf)
{
	QString output;


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
int64_t SurfaceMeshToSolidModelIges::SurfMeshParams(const int64_t n, const int64_t m, const double *Q, double *uk, double *vl)
{
	// This code adapted from The NURBS Handbook, 2nd Ed. ISBN 3-540-61545-8 p377-378

	// Calculate uk
	int64_t num = m+1;
	uk[0] = 0.0f;	uk[n] = 1.0f;
	double *cds = new double[n+1];
	for (int64_t k=1; k<n; ++k) uk[k] = 0.0f;
	for (int64_t l=0; l<=m; ++l)
	{
		double total = 0.0f;
		for (int64_t k=1; k<=n; ++k)
		{
			cds[k] = Distance3D(k,l,k-1,l,n,m,Q);
			total += cds[k];
		}
		if ( total <= DBL_EPSILON ) --num;
		else
		{
			double d = 0.0f;
			for (int64_t k=1; k<n; ++k)
			{
				d += cds[k];
				uk[k] += d/total;
			}
		}
	}
	if (num == 0) return -1;
	for (int64_t k=1; k<n; ++k) uk[k] = uk[k]/num;

	// Calculate vl
	num = n+1;
	vl[0] = 0.0f; vl[m] = 1.0f;
	delete[] cds;
	cds = new double[m+1];
	for (int64_t l=1; l<m; ++l) vl[l] = 0.0f;
	for (int64_t k=0; k<=n; ++k)
	{
		double total = 0.0f;
		for (int64_t l=1; l<=m; ++l)
		{
			cds[l] = Distance3D(k,l,k,l-1,n,m,Q);
			total += cds[l];
		}
		if (total <= DBL_EPSILON) --num;
		else
		{
			double d = 0.0f;
			for (int64_t l=1; l<m; ++l)
			{
				d += cds[l];
				vl[l] += d/total;
			}
		}
	}
	if (num == 0) return -1;
	for (int64_t l=1; l<m; ++l) vl[l] = vl[l]/num;

	delete[] cds;

	return 0;	
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
double SurfaceMeshToSolidModelIges::Distance3D(const int64_t xa, const int64_t ya, const int64_t xb, const int64_t yb, const int64_t n, const int64_t m, const double *Q)
{
	const double ax = Q[0 + 3 * ya + xa * (m + 1) * 3];
	const double ay = Q[1 + 3 * ya + xa * (m + 1) * 3];
	const double az = Q[2 + 3 * ya + xa * (m + 1) * 3];
	const double bx = Q[0 + 3 * yb + xb * (m + 1) * 3];
	const double by = Q[1 + 3 * yb + xb * (m + 1) * 3];
	const double bz = Q[2 + 3 * yb + xb * (m + 1) * 3];

	double retval = sqrt( (ax-bx)*(ax-bx) + (ay-by)*(ay-by) + (az-bz)*(az-bz) );
	
	return retval;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
double SurfaceMeshToSolidModelIges::Distance3D(const int64_t a, const int64_t b, const double *Q)
{
	const double ax = Q[0 + 3 * a];
	const double ay = Q[1 + 3 * a];
	const double az = Q[2 + 3 * a];
	const double bx = Q[0 + 3 * b];
	const double by = Q[1 + 3 * b];
	const double bz = Q[2 + 3 * b];

	double retval = sqrt((ax - bx)*(ax - bx) + (ay - by)*(ay - by) + (az - bz)*(az - bz));

	return retval;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::GlobalSurfInterp(int64_t n, int64_t m, const double *Q, int64_t p, int64_t q, double *U, double *V, double *P)
{
	double *uk = new double[n+1];
	double *vl = new double[m+1];
	
	SurfMeshParams(n,m,Q,uk,vl);

	// Compute U
	for (int64_t k = 0; k <= p; k++) U[k] = 0.0f;
	for (int64_t k = n+1;k <= n+p+1; k++) U[k] = 1.0f;
	for (int64_t j = 1; j <= n - p; j++)
	{
		double total = 0.0f;
		for (int64_t i = j; i <= j + p - 1; ++i) total += uk[i];
		U[j + p] = total / p;
	}

	// Compute V
	for (int64_t k = 0; k <= q; k++) V[k] = 0.0f;
	for (int64_t k = m+1;k <= m+q+1; k++) V[k] = 1.0f;
	for (int64_t j = 1; j <= m - q; j++)
	{
		double total = 0.0f;
		for (int64_t i = j; i <= j + q - 1; ++i) total += vl[i];
		V[j + q] = total / q;
	}


	double *A = new double[(n + 1)*(n + 1)];
	double *R = new double[(n+1)*(m+1)*3];
	int64_t *indx = new int64_t[n+1];
	double d = 0.0f;
	double lud = 0.0f;
	for (int64_t i = 0; i < (n + 1)*(n + 1); ++i) A[i] = 0.0f;

	for (int64_t i = 0; i <= n; ++i)
	{
		int64_t span = FindSpan(n, p, uk[i], U);
		BasisFunction(span, uk[i], p, U, &A[(i)*(n + 1) + (span-p)]);
	}
	ludcmp(A, n+1, indx, &d);
	double *rhs = new double[n+1];

	for (int64_t l=0; l<=m; l++)
	{

		for (int64_t i = 0; i < 3; i++)
		{
			for (int64_t j = 0; j <= n; j++) rhs[j] = Q[(m+1)*3*j + 3*(l) + (i)];
			lubksb(A, n + 1, indx, rhs);
			for (int64_t j = 0; j <= n; j++) R[(m+1)*3*j + 3*(l) + (i)] = rhs[j];
		}

	}
	
	delete[] A;
	delete[] indx;
	delete[] rhs;


	d = 0.0f;
	A = new double[(m + 1)*(m + 1)];
	indx = new int64_t[m+1];
	lud = 0.0f;
	for (int64_t i = 0; i < (m + 1)*(m + 1); ++i) A[i] = 0.0f;

	for (int64_t i = 0; i <= m; ++i)
	{
		int64_t span = FindSpan(m, q, vl[i], V);
		BasisFunction(span, vl[i], q, V, &A[(i)*(m + 1) + (span-q)]);
	}
	ludcmp(A, m + 1, indx, &d);
	rhs = new double[m+1];

	for (int64_t l=0; l<=n; l++)
	{

		for (int64_t i = 0; i < 3; i++)
		{
			for (int64_t j = 0; j <= m; j++) rhs[j] = R[(m+1)*3*l + 3*(m) + (i)];
			lubksb(A, m + 1, indx, rhs);
			for (int64_t j = 0; j <= n; j++) P[(m+1)*3*l + 3*(m) + (i)] = rhs[j];
		}

	}


	delete[] R;

	delete[] A;
	delete[] indx;
	delete[] rhs;

	
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::GlobalCurveInterp(int64_t n, double *Q, int64_t r, int64_t p, int64_t &m, double *U, double *P)
{
	// This code adapted from The NURBS Handbook, 2nd Ed. ISBN 3-540-61545-8 p369-370

	m = n + p + 1;

	double *uk = new double[n+1];
	uk[0] = 0.0f;  uk[n] = 1.0f;
	double *cds = new double[n+1];
	for (int64_t k = 1; k<n; ++k) uk[k] = 0.0f;
	double d = 0.0f;
	for (int64_t k = 1; k <= n; ++k)
	{
		cds[k] = Distance3D(k, k - 1, Q);
		d += cds[k];
	}
	for (int64_t k = 1; k<n; ++k)
	{
		uk[k] = uk[k-1] + (cds[k]/d);
	}
		
	for (int64_t k = 0; k <= p; k++) U[k] = 0.0f;
	for (int64_t k = m-p;k <= m; k++) U[k] = 1.0f;

	
	for (int64_t j = 1; j <= n - p; j++)
	{
		double total = 0.0f;
		for (int64_t i = j; i <= j + p - 1; ++i) total += uk[i];
		U[j + p] = total / p;
	}

	delete[] cds;

	double *A = new double[(n + 1)*(n+1)];
	int64_t *indx = new int64_t[n+1];
	double lud = 0.0f;
	for (int64_t i = 0; i < (n + 1)*(n + 1); ++i) A[i] = 0.0f;

	for (int64_t i = 0; i <= n; ++i)
	{
		int64_t span = FindSpan(n, p, uk[i], U);
		BasisFunction(span, uk[i], p, U, &A[(i)*(n + 1) + (span-p)]);
	}
	ludcmp(A, n + 1, indx, &d);
	double *rhs = new double[n+1];
	for (int64_t i = 0; i < r; i++)
	{
		for (int64_t j = 0; j <= n; j++) rhs[j] = Q[(j)*(r) + (i)];
		lubksb(A, n + 1, indx, rhs);
		for (int64_t j = 0; j <= n; j++) P[(j)*(r) + (i)] = rhs[j];
	}
	
	delete[] A;
	delete[] indx;
	delete[] uk;

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int64_t SurfaceMeshToSolidModelIges::FindSpan(int64_t n, int64_t p, double u, double *U)
{
	if (u == U[n+1]) return(n);

	int64_t low = p; int64_t high = n+1;

	int64_t mid = (low+high)/2;

	while (u < U[mid] || u >= U[mid+1])
	{
		if (u<U[mid]) high = mid;
		else	low = mid;
		mid = (low+high)/2;
	}
	return(mid);
	
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::BasisFunction(int64_t i, double u, int64_t p, double *U, double *N)
{

	N[0] = 1.0;
	double *left = new double[p+1];
	double *right = new double[p+1];
	double saved;


	for (int64_t j = 1; j<=p; j++)
	{
		left[j] = u-U[i+1-j];
		right[j] = U[i+j]-u;
		saved = 0.0;
		for (int64_t r=0; r<j; r++)
		{
			double temp = N[r]/(right[r+1] + left[j-r]);
			N[r] = saved + right[r+1] * temp;
			saved = left[j-r]*temp;
		}
		N[j] = saved;
	}

	delete[] left;
	delete[] right;

}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

#define SWAP(a,b) {dum=(a);(a)=(b);(b)=dum;}
#define TINY 1.0e-20

void SurfaceMeshToSolidModelIges::bandec(double *a, int64_t n, int64_t m1, int64_t m2, double *al, int64_t indx[], double *d)
{
	int64_t i,j,k,l;
	int64_t mm;
	double dum;	
	
	mm = m1 + m2 + 1;
	l = m1;
	for (i=1;i<=m1;i++)
	{
		for (j=m1+2-i;j<=mm;j++) a[n*(i)+(j-l)]=a[n*(i)+(j)];
		--l;
		for (j = mm - l; j <= mm; j++) a[n*(i) + (j)] = 0.0f;
	}
	*d=1.0;
	l = m1;
	for (k=1;k<=n;k++)
	{
		dum = a[n*(k) + (1)];
		i=k;
		if (l<n) l++;
		for (j=k+1; j<=l; j++)
		{
			if (abs(a[n*(j) + (1)]) > abs(dum))
			{
				dum = a[n*(j) + (1)];
				i=j;
			}
		}
		indx[k] = i;
		if (dum == 0.0) a[n*(k) + (1)] = TINY;
		if(i!=k) 
		{
			*d = -(*d);
			for (j = 1; j <= mm; j++) SWAP(a[n*(k)+(j)], a[n*(i) + (j)])
		}
		for (i=k+1;i<=l;i++)
		{
			dum = a[n*(i)+(1)] / a[n*(k) + (1)];
			al[n*(k) + (i-k)] = dum;
			for (j = 2; j <= mm; j++) a[n*(i)+(j - 1)] = a[n*(i)+(j)] - dum*a[n*(k) + (j)];
			a[n*(i) + (mm)] = 0.0;
		}
	}

}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::banbks(double *a, int64_t n, int64_t m1, int64_t m2, double *al, int64_t indx[], double b[])
{
	int64_t i,k,l;
	int64_t mm;
	double dum;

	mm = m1+m2+1;
	l = m1;

	for (k=1; k<=n; k++) 
	{
		i = indx[k];
		if (i != k) SWAP(b[k], b[i])
		if (l < n) l++;
		for (i = k + 1; i <= l; i++) b[i] -= al[n*(k) + (i-k)] * b[k];
	}

	l=1;

	for (i=n; i>=1; i--)
	{
		dum = b[i];
		for (k = 2; k <= l; k++) dum -= a[n*(i) + (k)] * b[k + i - 1];
		b[i] = dum / a[n*(i) + (1)];
		if (l<mm) l++;
	}
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::ludcmp(double *a, int64_t n, int64_t *indx, double *d)
{
	int64_t i, imax, j, k;
	double big, dum, sum, temp;
	double *vv;

	vv = new double[n];
	*d = 1.0f;
	for (i = 0; i < n; i++)
	{
		big = 0.0f;
		for (j = 0; j < n; j++)
			if ((temp = fabs(a[n*i + j]))>big) big = temp;
		if (big == 0.0) assert(false);
		vv[i] = 1.0 / big;
	}
	for (j = 0; j < n; j++)
	{
		for (i = 0; i < j - 1; i++)
		{
			sum = a[n*i + j];
			for (k = 0; k < j - 1; k++)	sum -= a[n*i + k] * a[n*k + j];
			a[n*i + j] = sum;
		}
		big = 0.0;
		for (i = j; i < n; i++) {
			sum = a[n*i + j];
			for (k = 0; k < j; k++)
				sum -= a[n*i + k] * a[n*k + j];
			a[n*i + j] = sum;
			if ((dum = vv[i] * fabs(sum)) >= big)
			{
				big = dum;
				imax = i;
			}
		}
		if (j != imax) {
			for (k = 0; k < n; k++)
			{
				dum = a[n*imax + k];
				a[n*imax + k] = a[n*j + k];
				a[n*j + k] = dum;
			}
			*d = -(*d);
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (a[n*j + j] == 0.0) a[n*j + j] = TINY;
		if (j != n) {
			dum = 1.0 / (a[n*j + j]);
			for (i = j; i < n; i++) a[n*i + j] *= dum;
		}
	}
	delete[] vv;
}



// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SurfaceMeshToSolidModelIges::lubksb(double *a, int64_t n, int64_t *indx, double *b)
{
	int64_t i, ii = 0, ip, j;
	double sum;

	for (i = 0; i < n; i++)
	{
		ip = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if (ii)
			for (j = ii; j <= i - 1; j++) sum -= a[n*i + j] * b[j];
		else if (sum) ii = i;
		b[i] = sum;
	}
	for (i = n-1; i >= 0; i--)
	{
		sum = b[i];
		for (j = i; j < n; j++) sum -= a[n*i + j] * b[j];
		b[i] = sum / a[n*i + i];
	}
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
QList<int32_t> SurfaceMeshToSolidModelIges::RecurseExternalTrianglesOnSurface(ElementDynamicList::Pointer m_TriangleNeighbors, QVector<bool> &checkedtriangles, int64_t t, int32_t grain, int32_t featurefacelabel, float n0, float n1, float n2)
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
			if (checkedtriangles[t] == false && m_SurfaceMeshFeatureFaceIds[t] == featurefacelabel && (m_SurfaceMeshFaceLabels[2*t] == grain || m_SurfaceMeshFaceLabels[2*t+1] == grain) && 
				m_SurfaceMeshTriangleNormals[3*t] == n0 && 
				m_SurfaceMeshTriangleNormals[3*t+1] == n1 &&
				m_SurfaceMeshTriangleNormals[3*t+2] == n2)
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
