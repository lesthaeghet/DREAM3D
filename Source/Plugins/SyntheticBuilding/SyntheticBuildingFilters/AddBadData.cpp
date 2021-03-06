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


#include "AddBadData.h"

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/LinkedBooleanFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"
#include "DREAM3DLib/Utilities/DREAM3DRandom.h"

#include "SyntheticBuilding/SyntheticBuildingConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AddBadData::AddBadData() :
  AbstractFilter(),
  m_GBEuclideanDistancesArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::ElementAttributeMatrixName, DREAM3D::CellData::GBEuclideanDistances),
  m_PoissonNoise(false),
  m_PoissonVolFraction(0.0f),
  m_BoundaryNoise(false),
  m_BoundaryVolFraction(0.0f),
  m_GBEuclideanDistances(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AddBadData::~AddBadData()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AddBadData::setupFilterParameters()
{
  FilterParameterVector parameters;
  QStringList linkedProps("PoissonVolFraction");
  parameters.push_back(LinkedBooleanFilterParameter::New("Add Random Noise", "PoissonNoise", getPoissonNoise(), linkedProps, FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Volume Fraction of Random Noise", "PoissonVolFraction", FilterParameterWidgetType::DoubleWidget, getPoissonVolFraction(), FilterParameter::Parameter));
  linkedProps.clear();
  linkedProps << "BoundaryVolFraction";
  parameters.push_back(LinkedBooleanFilterParameter::New("Add Boundary Noise", "BoundaryNoise", getBoundaryNoise(), linkedProps, FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Volume Fraction of Boundary Noise", "BoundaryVolFraction", FilterParameterWidgetType::DoubleWidget, getBoundaryVolFraction(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Feature Boundary Euclidean Distances", "GBEuclideanDistancesArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getGBEuclideanDistancesArrayPath(), FilterParameter::RequiredArray, ""));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AddBadData::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setGBEuclideanDistancesArrayPath(reader->readDataArrayPath("GBEuclideanDistancesArrayPath", getGBEuclideanDistancesArrayPath() ) );
  setPoissonNoise( reader->readValue("PoissonNoise", getPoissonNoise()) );
  setPoissonVolFraction( reader->readValue("PoissonVolFraction", getPoissonVolFraction()) );
  setBoundaryNoise( reader->readValue("BoundaryNoise", getBoundaryNoise()) );
  setBoundaryVolFraction( reader->readValue("BoundaryVolFraction", getBoundaryVolFraction()) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AddBadData::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(GBEuclideanDistancesArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(PoissonNoise)
  DREAM3D_FILTER_WRITE_PARAMETER(PoissonVolFraction)
  DREAM3D_FILTER_WRITE_PARAMETER(BoundaryNoise)
  DREAM3D_FILTER_WRITE_PARAMETER(BoundaryVolFraction)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AddBadData::dataCheck()
{
  setErrorCondition(0);

  QVector<size_t> cDims(1, 1);
  m_GBEuclideanDistancesPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getGBEuclideanDistancesArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_GBEuclideanDistancesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_GBEuclideanDistances = m_GBEuclideanDistancesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AddBadData::preflight()
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
void AddBadData::execute()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  add_noise();

  // If there is an error set this to something negative and also set a message
  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void  AddBadData::add_noise()
{
  notifyStatusMessage(getHumanLabel(), "Adding Noise");
  DREAM3D_RANDOMNG_NEW()

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getGBEuclideanDistancesArrayPath().getDataContainerName());

  QString attMatName = getGBEuclideanDistancesArrayPath().getAttributeMatrixName();
  QList<QString> voxelArrayNames = m->getAttributeMatrix(attMatName)->getAttributeArrayNames();

  float random = 0.0f;
  size_t totalPoints = m->getGeometryAs<ImageGeom>()->getNumberOfElements();
  for (size_t i = 0; i < totalPoints; ++i)
  {
    if (m_BoundaryNoise == true && m_GBEuclideanDistances[i] < 1)
    {
      random = static_cast<float>( rg.genrand_res53() );
      if (random < m_BoundaryVolFraction)
      {
        for (QList<QString>::iterator iter = voxelArrayNames.begin(); iter != voxelArrayNames.end(); ++iter)
        {
          IDataArray::Pointer p = m->getAttributeMatrix(attMatName)->getAttributeArray(*iter);
          p->initializeTuple(i, 0);
        }
      }
    }
    if (m_PoissonNoise == true)
    {
      random = static_cast<float>( rg.genrand_res53() );
      if (random < m_PoissonVolFraction)
      {
        for (QList<QString>::iterator iter = voxelArrayNames.begin(); iter != voxelArrayNames.end(); ++iter)
        {
          IDataArray::Pointer p = m->getAttributeMatrix(attMatName)->getAttributeArray(*iter);
          p->initializeTuple(i, 0);
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer AddBadData::newFilterInstance(bool copyFilterParameters)
{
  AddBadData::Pointer filter = AddBadData::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AddBadData::getCompiledLibraryName()
{ return SyntheticBuildingConstants::SyntheticBuildingBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AddBadData::getGroupName()
{ return DREAM3D::FilterGroups::SyntheticBuildingFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AddBadData::getSubGroupName()
{ return DREAM3D::FilterSubGroups::MiscFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString AddBadData::getHumanLabel()
{ return "Add Bad Data"; }
