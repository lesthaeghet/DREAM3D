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
			internalSurfaceTriangles << QList<int32_t>();
			// Let's look for matching triangles
			for (int64_t t = 0; t < nTriangles; ++t)
			{
				// Have we found a triangle on this grain and surface
				if (m_SurfaceMeshFeatureFaceIds[t] == uniqueInternalSurfaces.at(facecnt) && m_SurfaceMeshFaceLabels[t] == grain)
				{
					// Let's store this triangle
					internalSurfaceTriangles[facecnt] << t;
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
				if (m_SurfaceMeshFeatureFaceIds[t] == externalSurfaceFaceLabel && m_SurfaceMeshFaceLabels[t] == grain)
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
				surfnorm[0] += m_SurfaceMeshTriangleNormals[3 * (*t) + 0];
				surfnorm[1] += m_SurfaceMeshTriangleNormals[3 * (*t) + 1];
				surfnorm[2] += m_SurfaceMeshTriangleNormals[3 * (*t) + 2];
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



			float pt1[3] = {ll[0], ll[1], ll[2]};
			float pt2[3] = {ll[0], ll[1], ur[2]};
			float pt3[3] = {ll[0], ur[1], ur[2]};
			float pt4[3] = {ll[0], ur[1], ll[2]};
			float pt5[3] = {ur[0], ur[1], ur[2]};
			float pt6[3] = {ur[0], ur[1], ll[2]};
			float pt7[3] = {ur[0], ll[1], ll[2]};
			float pt8[3] = {ur[0], ll[1], ur[2]};
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
			float ydim = ur[1] - ll[0];

			int32_t xcnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * xdim / ydim)));
			int32_t ycnt = static_cast <int32_t> (ceil(sqrt((*cursurf).size() * ydim / xdim)));

			float *griddedsurf = new float[xcnt*ycnt*3];
			

			std::vector<float> xvals = DREAM3DMath::linspace(ll[0], ur[0], xcnt);
			std::vector<float> yvals = DREAM3DMath::linspace(ll[1], ur[1], ycnt);

			float tmppt1[3];
			float tmpptr1[3];
			float tmppt2[3];
			float tmpptr2[3];
			tmppt1[2] = ll[2];
			tmppt2[2] = ur[2];


			// We can now start directly searching for grid values
			for (int32_t i = 0; i < xcnt; i++)	
			{
				for (int32_t j = 0; j < ycnt; j++)
				{
					QString ss = QObject::tr("Ray Tracing Grid Point %1/%2").arg(ycnt*i + j).arg(xcnt*ycnt);
					notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);

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

						code = GeometryMath::RayIntersectsTriangle(vert1, vert2, vert3, tmpptr1, tmpptr2, newgridpt);

						// We've got a match and a point
						if (code == 'f' || code == 'v' || code == 'e' || code == 'E' || code == 'V' || code == 'F')
							break;

					}

					// Make sure we actually had a match and set NAN if we didn't
					if (code != 'f' && code != 'v' && code != 'e' && code != 'E' && code != 'V' && code != 'F')
					{
						newgridpt[0] = NAN; newgridpt[1] = NAN; newgridpt[2] = NAN;
					}

					// Save the grid point value
					griddedsurf[0 + j * 3 + i*ycnt * 3] = newgridpt[0];
					griddedsurf[1 + j * 3 + i*ycnt * 3] = newgridpt[1];
					griddedsurf[2 + j * 3 + i*ycnt * 3] = newgridpt[2];

				}
			}

			// Great!  Now we've managed to fit the grid down onto the surface
			// However, there are very likely still interior points that weren't
			// found if the triangle at that point happens to be almost perfectly
			// parallel to the ray.  We should have enough resolution that all we
			// need to do is interpolate the grid and call it good.  We also have
			// the problem of dealing with the outer edges.



			// Let's find a list of vertex points that map to the edge of the
			// surface from the list of triangles.
			int64_t curfeaturefaceid = m_SurfaceMeshFeatureFaceIds[(*cursurf)[0]];
			QList<QList<int64_t>> edgelist;
			QMap<int64_t, int32_t> edgevertexmap;
			for (QList<int32_t>::iterator t = (*cursurf).begin(); t != (*cursurf).end(); ++t)
			{
				// Get the list of neighbors for this triangle
				uint16_t neighborcount = m_TriangleNeighbors->getNumberOfElements(*t);
				int64_t* nList = m_TriangleNeighbors->getElementListPointer(*t);

				// For each neighbor
				for (uint16_t i = 0; i < neighborcount; ++i)
				{
					if (m_SurfaceMeshFaceLabels[nList[i] * 2] == grain || m_SurfaceMeshFaceLabels[nList[i] * 2 + 1] == grain)
					{
						if (m_SurfaceMeshFeatureFaceIds[nList[i]] != curfeaturefaceid)
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
										edgevertexmap.insert(a[c], 0);
									}
								}
							}

							edgelist << outvals;
							
							
							
						}
					}
				}
			}


			// Temporary surface output
			QString filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
			filename = filename + QString("Grain_") + QString::number(grain) + QString("Surface_") + QString::number(surfcnt) + QString("_Size") + QString::number(xcnt) + QString("x") + QString::number(ycnt) + ".csv";
			FILE* f = fopen(filename.toLatin1().data(), "w");
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

			// Temporary edge output
			filename = getOutputIgesDirectory() + "/" + getOutputIgesPrefix();
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
			fclose(f);






			delete[] griddedsurf;

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
