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


#include "AlignSectionsFeatureCentroid.h"

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/LinkedBooleanFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"

#include "Reconstruction/ReconstructionConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AlignSectionsFeatureCentroid::AlignSectionsFeatureCentroid() :
  AlignSections(),
  m_ReferenceSlice(0),
  m_UseReferenceSlice(false),
  m_GoodVoxelsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::CellAttributeMatrixName, DREAM3D::CellData::GoodVoxels),
  m_GoodVoxels(NULL)
{
  // only setting up the child parameters because the parent constructor has already been called
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AlignSectionsFeatureCentroid::~AlignSectionsFeatureCentroid()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::setupFilterParameters()
{
  // getting the current parameters that were set by the parent and adding to it before resetting it
  FilterParameterVector parameters = getFilterParameters();

  QStringList linkedProps("ReferenceSlice");
  parameters.push_back(LinkedBooleanFilterParameter::New("Use Reference Slice", "UseReferenceSlice", getUseReferenceSlice(), linkedProps, FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Reference Slice", "ReferenceSlice", FilterParameterWidgetType::IntWidget, getReferenceSlice(), FilterParameter::Parameter));

  parameters.push_back(FilterParameter::New("Good Voxels", "GoodVoxelsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getGoodVoxelsArrayPath(), FilterParameter::RequiredArray, ""));

  setFilterParameters(parameters);
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  AlignSections::readFilterParameters(reader, index);
  reader->openFilterGroup(this, index);
  setGoodVoxelsArrayPath(reader->readDataArrayPath("GoodVoxelsArrayPath", getGoodVoxelsArrayPath() ) );
  setUseReferenceSlice( reader->readValue("UseReferenceSlice", getUseReferenceSlice()) );
  setReferenceSlice( reader->readValue("ReferenceSlice", getReferenceSlice()) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AlignSectionsFeatureCentroid::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  AlignSections::writeFilterParameters(writer, index);
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(GoodVoxelsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(UseReferenceSlice)
  DREAM3D_FILTER_WRITE_PARAMETER(ReferenceSlice)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::dataCheck()
{
  setErrorCondition(0);

  // Set the DataContainerName and AttributematrixName for the Parent Class (AlignSections) to Use.
  // These are checked for validity in the Parent Class dataCheck
  setDataContainerName(m_GoodVoxelsArrayPath.getDataContainerName());
  setCellAttributeMatrixName(m_GoodVoxelsArrayPath.getAttributeMatrixName());

  ImageGeom::Pointer image = getDataContainerArray()->getPrereqGeometryFromDataContainer<ImageGeom, AbstractFilter>(this, m_GoodVoxelsArrayPath.getDataContainerName());
  if(getErrorCondition() < 0) { return; }

  if (m_ReferenceSlice > image->getZPoints())
  {
    QString ss = QObject::tr("The Image Geometry extent (%1) is smaller than the supplied reference slice (%2)").arg(image->getZPoints()).arg(m_ReferenceSlice);
    setErrorCondition(-5556);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  QVector<size_t> cDims(1, 1);
  m_GoodVoxelsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<bool>, AbstractFilter>(this, getGoodVoxelsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_GoodVoxelsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_GoodVoxels = m_GoodVoxelsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  if(getErrorCondition() < 0) { return; }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::preflight()
{
  setInPreflight(true);
  emit preflightAboutToExecute();
  emit updateFilterParameters(this);
  dataCheck();
  emit preflightExecuted();
  AlignSections::preflight();
  setInPreflight(false);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::find_shifts(std::vector<int64_t>& xshifts, std::vector<int64_t>& yshifts)
{
  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getDataContainerName());

  std::ofstream outFile;
  if (getWriteAlignmentShifts() == true)
  {
    outFile.open(getAlignmentShiftFileName().toLatin1().data());
  }
  size_t udims[3] = { 0, 0, 0 };
  m->getGeometryAs<ImageGeom>()->getDimensions(udims);
#if (CMP_SIZEOF_SIZE_T == 4)
  typedef int32_t DimType;
#else
  typedef int64_t DimType;
#endif
  DimType dims[3] =
  {
    static_cast<DimType>(udims[0]),
    static_cast<DimType>(udims[1]),
    static_cast<DimType>(udims[2]),
  };

  int64_t newxshift = 0;
  int64_t newyshift = 0;
  int64_t count = 0;
  DimType slice = 0;
  DimType point = 0;
  float xRes = m->getGeometryAs<ImageGeom>()->getXRes();
  float yRes = m->getGeometryAs<ImageGeom>()->getYRes();
  std::vector<float> xCentroid(dims[2], 0.0f);
  std::vector<float> yCentroid(dims[2], 0.0f);

  for (DimType iter = 0; iter < dims[2]; iter++)
  {
    count = 0;
    xCentroid[iter] = 0;
    yCentroid[iter] = 0;

    QString ss = QObject::tr("Aligning Sections - Determining Shifts - %1 Percent Complete").arg(((float)iter / dims[2]) * 100);
    notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);
    slice = static_cast<int>( (dims[2] - 1) - iter );
    for (DimType l = 0; l < dims[1]; l++)
    {
      for (DimType n = 0; n < dims[0]; n++)
      {
        point = ((slice) * dims[0] * dims[1]) + (l * dims[0]) + n;
        if (m_GoodVoxels[point] == true)
        {
          xCentroid[iter] = xCentroid[iter] + (float(n) * xRes);
          yCentroid[iter] = yCentroid[iter] + (float(l) * yRes);
          count++;
        }
      }
    }
    xCentroid[iter] = xCentroid[iter] / float(count);
    yCentroid[iter] = yCentroid[iter] / float(count);
  }
  for (DimType iter = 1; iter < dims[2]; iter++)
  {
    slice = (dims[2] - 1) - iter;
    if (m_UseReferenceSlice == true)
    {
      xshifts[iter] = static_cast<int64_t>((xCentroid[iter] - xCentroid[m_ReferenceSlice]) / xRes);
      yshifts[iter] = static_cast<int64_t>((yCentroid[iter] - yCentroid[m_ReferenceSlice]) / yRes);
    }
    else
    {
      xshifts[iter] = xshifts[iter - 1] + static_cast<int64_t>((xCentroid[iter] - xCentroid[iter - 1]) / xRes);
      yshifts[iter] = yshifts[iter - 1] + static_cast<int64_t>((yCentroid[iter] - yCentroid[iter - 1]) / yRes);
    }
    if (getWriteAlignmentShifts() == true)
    {
      outFile << slice << "	" << slice + 1 << "	" << newxshift << "	" << newyshift << "	" << xshifts[iter] << "	" << yshifts[iter] << " " << xCentroid[iter] << " " << yCentroid[iter] << std::endl;
    }
  }
  if (getWriteAlignmentShifts() == true)
  {
    outFile.close();
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AlignSectionsFeatureCentroid::execute()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  AlignSections::execute();

  // If there is an error set this to something negative and also set a message
  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer AlignSectionsFeatureCentroid::newFilterInstance(bool copyFilterParameters)
{
  AlignSectionsFeatureCentroid::Pointer filter = AlignSectionsFeatureCentroid::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AlignSectionsFeatureCentroid::getCompiledLibraryName()
{ return ReconstructionConstants::ReconstructionBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AlignSectionsFeatureCentroid::getGroupName()
{ return DREAM3D::FilterGroups::ReconstructionFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AlignSectionsFeatureCentroid::getSubGroupName()
{return DREAM3D::FilterSubGroups::AlignmentFilters;}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AlignSectionsFeatureCentroid::getHumanLabel()
{ return "Align Sections (Feature Centroid)"; }
