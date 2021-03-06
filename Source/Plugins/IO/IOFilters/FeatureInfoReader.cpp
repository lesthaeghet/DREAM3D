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


#include "FeatureInfoReader.h"

#include <QtCore/QFileInfo>

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/FileSystemFilterParameter.h"
#include "DREAM3DLib/FilterParameters/LinkedBooleanFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"

#include "IO/IOConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
FeatureInfoReader::FeatureInfoReader() :
  FileReader(),
  m_CellAttributeMatrixName(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::CellAttributeMatrixName, ""),
  m_CellFeatureAttributeMatrixName(DREAM3D::Defaults::CellFeatureAttributeMatrixName),
  m_InputFile(""),
  m_CreateCellLevelArrays(true),
  m_RenumberFeatures(true),
  m_FeatureIdsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::CellAttributeMatrixName, DREAM3D::CellData::FeatureIds),
  m_CellPhasesArrayName(DREAM3D::CellData::Phases),
  m_CellEulerAnglesArrayName(DREAM3D::CellData::EulerAngles),
  m_FeaturePhasesArrayName(DREAM3D::FeatureData::Phases),
  m_FeatureEulerAnglesArrayName(DREAM3D::FeatureData::EulerAngles),
  m_FeatureIds(NULL),
  m_CellPhases(NULL),
  m_CellEulerAngles(NULL),
  m_FeaturePhases(NULL),
  m_FeatureEulerAngles(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
FeatureInfoReader::~FeatureInfoReader()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FeatureInfoReader::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(FileSystemFilterParameter::New("Input Feature Info File", "InputFile", FilterParameterWidgetType::InputFileWidget, getInputFile(), FilterParameter::Parameter, "", "*.txt"));
  QStringList linkedProps;
  linkedProps << "CellPhasesArrayName" << "CellEulerAnglesArrayName";
  parameters.push_back(LinkedBooleanFilterParameter::New("Create Element Level Arrays", "CreateCellLevelArrays", getCreateCellLevelArrays(), linkedProps, FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Renumber Features", "RenumberFeatures", FilterParameterWidgetType::BooleanWidget, getRenumberFeatures(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Element Attribute Matrix", "CellAttributeMatrixName", FilterParameterWidgetType::AttributeMatrixSelectionWidget, getCellAttributeMatrixName(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Element Feature Ids", "FeatureIdsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getFeatureIdsArrayPath(), FilterParameter::RequiredArray, ""));
  parameters.push_back(FilterParameter::New("Element Feature Attribute Matrix", "CellFeatureAttributeMatrixName", FilterParameterWidgetType::StringWidget, getCellFeatureAttributeMatrixName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Element Phases", "CellPhasesArrayName", FilterParameterWidgetType::StringWidget, getCellPhasesArrayName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Element Euler Angles", "CellEulerAnglesArrayName", FilterParameterWidgetType::StringWidget, getCellEulerAnglesArrayName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Feature Phases", "FeaturePhasesArrayName", FilterParameterWidgetType::StringWidget, getFeaturePhasesArrayName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Feature EulerAngles", "FeatureEulerAnglesArrayName", FilterParameterWidgetType::StringWidget, getFeatureEulerAnglesArrayName(), FilterParameter::CreatedArray, ""));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FeatureInfoReader::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setCellAttributeMatrixName(reader->readDataArrayPath("CellAttributeMatrixName", getCellAttributeMatrixName() ) );
  setCellFeatureAttributeMatrixName(reader->readString("CellFeatureAttributeMatrixName", getCellFeatureAttributeMatrixName() ) );
  setFeatureEulerAnglesArrayName(reader->readString("FeatureEulerAnglesArrayName", getFeatureEulerAnglesArrayName() ) );
  setFeaturePhasesArrayName(reader->readString("FeaturePhasesArrayName", getFeaturePhasesArrayName() ) );
  setCellEulerAnglesArrayName(reader->readString("CellEulerAnglesArrayName", getCellEulerAnglesArrayName() ) );
  setCellPhasesArrayName(reader->readString("CellPhasesArrayName", getCellPhasesArrayName() ) );
  setFeatureIdsArrayPath(reader->readDataArrayPath("FeatureIdsArrayPath", getFeatureIdsArrayPath() ) );
  setInputFile( reader->readString( "InputFile", getInputFile() ) );
  setCreateCellLevelArrays( reader->readValue("CreateCellLevelArrays", getCreateCellLevelArrays()) );
  setRenumberFeatures( reader->readValue("RenumberFeatures", getRenumberFeatures()) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int FeatureInfoReader::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(CellAttributeMatrixName)
  DREAM3D_FILTER_WRITE_PARAMETER(CellFeatureAttributeMatrixName)
  DREAM3D_FILTER_WRITE_PARAMETER(FeatureEulerAnglesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(FeaturePhasesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(CellEulerAnglesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(CellPhasesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(FeatureIdsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(InputFile)
  DREAM3D_FILTER_WRITE_PARAMETER(CreateCellLevelArrays)
  DREAM3D_FILTER_WRITE_PARAMETER(RenumberFeatures)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FeatureInfoReader::updateFeatureInstancePointers()
{
  setErrorCondition(0);

  if( NULL != m_FeaturePhasesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeaturePhases = m_FeaturePhasesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  if( NULL != m_FeatureEulerAnglesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeatureEulerAngles = m_FeatureEulerAnglesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FeatureInfoReader::dataCheck()
{
  setErrorCondition(0);
  DataArrayPath tempPath;

  DataContainer::Pointer m = getDataContainerArray()->getPrereqDataContainer<AbstractFilter>(this, m_FeatureIdsArrayPath.getDataContainerName());
  if(getErrorCondition() < 0) { return; }

  QVector<size_t> tDims(1, 0);
  m->createNonPrereqAttributeMatrix<AbstractFilter>(this, getCellFeatureAttributeMatrixName(), tDims, DREAM3D::AttributeMatrixType::CellFeature);

  QFileInfo fi(getInputFile());
  if (getInputFile().isEmpty() == true)
  {
    QString ss = QObject::tr("The input file must be set").arg(ClassName());
    setErrorCondition(-387);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }
  else if (fi.exists() == false)
  {
    QString ss = QObject::tr("The input file does not exist");
    setErrorCondition(-388);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  QVector<size_t> cDims(1, 1);
  m_FeatureIdsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getFeatureIdsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_FeatureIdsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeatureIds = m_FeatureIdsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

  if (m_CreateCellLevelArrays)
  {
    tempPath.update(m_FeatureIdsArrayPath.getDataContainerName(), m_FeatureIdsArrayPath.getAttributeMatrixName(), getCellPhasesArrayName() );
    m_CellPhasesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter, int32_t>(this,  tempPath, 0, cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_CellPhasesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    { m_CellPhases = m_CellPhasesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

    cDims[0] = 3;
    tempPath.update(m_FeatureIdsArrayPath.getDataContainerName(), m_FeatureIdsArrayPath.getAttributeMatrixName(), getCellEulerAnglesArrayName() );
    m_CellEulerAnglesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this,  tempPath, 0, cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_CellEulerAnglesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    { m_CellEulerAngles = m_CellEulerAnglesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  }

  cDims[0] = 1;
  tempPath.update(m_FeatureIdsArrayPath.getDataContainerName(), getCellFeatureAttributeMatrixName(), getFeaturePhasesArrayName() );
  m_FeaturePhasesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter, int32_t>(this,  tempPath, 0, cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_FeaturePhasesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeaturePhases = m_FeaturePhasesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

  cDims[0] = 3;
  tempPath.update(m_FeatureIdsArrayPath.getDataContainerName(), getCellFeatureAttributeMatrixName(), getFeatureEulerAnglesArrayName() );
  m_FeatureEulerAnglesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this,  tempPath, 0, cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_FeatureEulerAnglesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeatureEulerAngles = m_FeatureEulerAnglesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void FeatureInfoReader::preflight()
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
int32_t FeatureInfoReader::readHeader()
{
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t FeatureInfoReader::readFile()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return getErrorCondition(); }

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(m_FeatureIdsArrayPath.getDataContainerName());
  AttributeMatrix::Pointer cellFeatureAttrMat = m->getAttributeMatrix(getCellFeatureAttributeMatrixName());

  std::ifstream inFile;
  inFile.open(getInputFile().toLatin1().data(), std::ios_base::binary);
  if (!inFile)
  {
    QString ss = QObject::tr("Error opening input file: %1").arg(getInputFile());
    setErrorCondition(-1);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return getErrorCondition();
  }

  int32_t numfeatures = 0;
  int32_t gnum = 0, phase = 0;
  int32_t maxphase = 0;
  float ea1 = 0.0f, ea2 = 0.0f, ea3 = 0.0f;
  inFile >> numfeatures;
  if (0 == numfeatures)
  {
    QString ss = QObject::tr("The number of Features (%1) specified in the file must be greater than zero").arg(numfeatures);
    setErrorCondition(-600);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return getErrorCondition();
  }

  size_t totalPoints = m_FeatureIdsPtr.lock()->getNumberOfTuples();

  int32_t maxFeatureId = 0;
  for (size_t i = 0; i < totalPoints; i++)
  {
    if (m_FeatureIds[i] > maxFeatureId)
    {
      maxFeatureId = m_FeatureIds[i];
    }
  }

  if (numfeatures != maxFeatureId)
  {
    QString ss = QObject::tr("The number of Features (%1) specified in the file does not correspond to the maximum Feature Id (%2) in the selected Feature Ids array").arg(numfeatures).arg(maxFeatureId);
    setErrorCondition(-600);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return getErrorCondition();
  }

  QVector<size_t> tDims(1, numfeatures + 1);
  cellFeatureAttrMat->setTupleDimensions(tDims);
  updateFeatureInstancePointers();

  for (int32_t i = 0; i < numfeatures; i++)
  {
    inFile >> gnum >> phase >> ea1 >> ea2 >> ea3;
    if (gnum >= maxFeatureId)
    {
      QString ss = QObject::tr("A Feature Id (%1) specified in the file is larger than the maximum Feature Id (%2) in the selected Feature Ids array").arg(numfeatures).arg(maxFeatureId);
      setErrorCondition(-600);
      notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
      return getErrorCondition();
    }
    m_FeatureEulerAngles[3 * gnum] = ea1;
    m_FeatureEulerAngles[3 * gnum + 1] = ea2;
    m_FeatureEulerAngles[3 * gnum + 2] = ea3;
    m_FeaturePhases[gnum] = phase;
    if (phase > maxphase) { maxphase = phase; }
  }

  if (m_CreateCellLevelArrays == true)
  {
    for (size_t i = 0; i < totalPoints; i++)
    {
      gnum = m_FeatureIds[i];
      m_CellEulerAngles[3 * i] = m_FeatureEulerAngles[3 * gnum];
      m_CellEulerAngles[3 * i + 1] = m_FeatureEulerAngles[3 * gnum + 1];
      m_CellEulerAngles[3 * i + 2] = m_FeatureEulerAngles[3 * gnum + 2];
      m_CellPhases[i] = m_FeaturePhases[gnum];
    }
  }

  if (m_RenumberFeatures == true)
  {
    size_t totalFeatures = cellFeatureAttrMat->getNumTuples();

    // Find the unique set of feature ids
    QVector<bool> activeObjects(totalFeatures, false);
    for (size_t i = 0; i < totalPoints; ++i)
    {
      activeObjects[m_FeatureIds[i]] = true;
    }
    cellFeatureAttrMat->removeInactiveObjects(activeObjects, m_FeatureIdsPtr.lock());
  }

  notifyStatusMessage(getHumanLabel(), "Complete");
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer FeatureInfoReader::newFilterInstance(bool copyFilterParameters)
{
  FeatureInfoReader::Pointer filter = FeatureInfoReader::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString FeatureInfoReader::getCompiledLibraryName()
{ return IOConstants::IOBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString FeatureInfoReader::getGroupName()
{ return DREAM3D::FilterGroups::IOFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString FeatureInfoReader::getSubGroupName()
{ return DREAM3D::FilterSubGroups::InputFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString FeatureInfoReader::getHumanLabel()
{ return "Read Feature Info File"; }
