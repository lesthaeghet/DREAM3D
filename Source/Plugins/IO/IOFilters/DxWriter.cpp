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


#include "DxWriter.h"

#include <QtCore/QDir>

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/FileSystemFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"

#include "IO/IOConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DxWriter::DxWriter() :
  FileWriter(),
  m_AddSurfaceLayer(false),
  m_FeatureIdsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::CellAttributeMatrixName, DREAM3D::CellData::FeatureIds),
  m_FeatureIds(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DxWriter::~DxWriter()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DxWriter::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(FileSystemFilterParameter::New("Output File", "OutputFile", FilterParameterWidgetType::OutputFileWidget, getOutputFile(), FilterParameter::Parameter, "", "*.dx", "Open DX Visualization"));
  parameters.push_back(FilterParameter::New("Add Surface Layer", "AddSurfaceLayer", FilterParameterWidgetType::BooleanWidget, getAddSurfaceLayer(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Cell Feature Ids", "FeatureIdsArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getFeatureIdsArrayPath(), FilterParameter::RequiredArray, ""));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DxWriter::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setFeatureIdsArrayPath(reader->readDataArrayPath("FeatureIdsArrayPath", getFeatureIdsArrayPath() ) );
  setOutputFile( reader->readString( "OutputFile", getOutputFile() ) );
  setAddSurfaceLayer( reader->readValue("AddSurfaceLayer", getAddSurfaceLayer()) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int DxWriter::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(FeatureIdsArrayPath)
  DREAM3D_FILTER_WRITE_PARAMETER(OutputFile)
  DREAM3D_FILTER_WRITE_PARAMETER(AddSurfaceLayer)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DxWriter::dataCheck()
{
  setErrorCondition(0);

  ImageGeom::Pointer image = getDataContainerArray()->getPrereqGeometryFromDataContainer<ImageGeom, AbstractFilter>(this, getFeatureIdsArrayPath().getDataContainerName());

  if (getOutputFile().isEmpty() == true)
  {
    QString ss = QObject::tr("The output file must be set");
    notifyErrorMessage(getHumanLabel(), ss, -1000);
    setErrorCondition(-1);
  }

  QFileInfo fi(getOutputFile());

  QDir parentPath = fi.path();
  if (parentPath.exists() == false)
  {
    QString ss = QObject::tr("The directory path for the output file does not exist");
    notifyWarningMessage(getHumanLabel(), ss, -1);
  }

  if (fi.suffix().compare("") == 0)
  {
    setOutputFile(getOutputFile().append(".dx"));
  }

  QVector<size_t> cDims(1, 1);
  m_FeatureIdsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getFeatureIdsArrayPath(), cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_FeatureIdsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_FeatureIds = m_FeatureIdsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

  if(getErrorCondition() < 0) { return; }

  size_t volTuples = image->getNumberOfElements();

  if (volTuples != m_FeatureIdsPtr.lock()->getNumberOfTuples() )
  {
    setErrorCondition(-10200);
    QString ss = QObject::tr("The number of Tuples for the DataArray %1 is %2 and for the associated Image Geometry is %3. The number of tuples must match.").arg(m_FeatureIdsPtr.lock()->getName()).arg(m_FeatureIdsPtr.lock()->getNumberOfTuples());
                                                                                                notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DxWriter::preflight()
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
int32_t DxWriter::writeHeader()
{
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int32_t DxWriter::writeFile()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return getErrorCondition(); }

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(m_FeatureIdsArrayPath.getDataContainerName());

  int32_t err = 0;
  size_t udims[3] =
  { 0, 0, 0 };
  m->getGeometryAs<ImageGeom>()->getDimensions(udims);
#if (CMP_SIZEOF_SIZE_T == 4)
  typedef int32_t DimType;
#else
  typedef int64_t DimType;
#endif
  DimType dims[3] =
  { static_cast<DimType>(udims[0]), static_cast<DimType>(udims[1]), static_cast<DimType>(udims[2]), };

  // Make sure any directory path is also available as the user may have just typed
  // in a path without actually creating the full path
  QFileInfo fi(getOutputFile());

  QDir dir(fi.path());
  if(!dir.mkpath("."))
  {
    QString ss;
    ss = QObject::tr("Error creating parent path '%1'").arg(dir.path());
    notifyErrorMessage(getHumanLabel(), ss, -1);
    setErrorCondition(-1);
    return -1;
  }

  QFile file(getOutputFile());
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    QString ss = QObject::tr("Error opening output file '%1'").arg(getOutputFile());
    setErrorCondition(-100);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return getErrorCondition();
  }

  QTextStream out(&file);
  DimType fileXDim = dims[0];
  DimType fileYDim = dims[1];
  DimType fileZDim = dims[2];

  DimType posXDim = fileXDim + 1;
  DimType posYDim = fileYDim + 1;
  DimType posZDim = fileZDim + 1;

  if (m_AddSurfaceLayer)
  {
    fileXDim = dims[0] + 2;
    fileYDim = dims[1] + 2;
    fileZDim = dims[2] + 2;

    posXDim = fileXDim + 1;
    posYDim = fileYDim + 1;
    posZDim = fileZDim + 1;
  }

  // Write the header
  out << "object 1 class gridpositions counts " << posZDim << " " << posYDim << " " << posXDim << "\n";
  out << "origin 0 0 0" << "\n";
  out << "delta  1 0 0" << "\n";
  out << "delta  0 1 0" << "\n";
  out << "delta  0 0 1" << "\n";
  out << "\n";
  out << "object 2 class gridconnections counts " << posZDim << " " << posYDim << " " << posXDim << "\n";
  out << "\n";
  out << "object 3 class array type int rank 0 items " << fileXDim* fileYDim* fileZDim << " data follows" << "\n";

  // Add a complete layer of surface voxels
  size_t rnIndex = 1;
  if (m_AddSurfaceLayer)
  {
    for (DimType i = 0; i < (fileXDim * fileYDim); ++i)
    {
      out << "-3 ";
      if (rnIndex == 20)
      {
        rnIndex = 0;
        out << "\n";
      }
      rnIndex++;
    }
  }

  DimType index = 0;
  for (DimType z = 0; z < dims[2]; ++z)
  {
    // Add a leading surface Row for this plane if needed
    if (m_AddSurfaceLayer)
    {
      for (DimType i = 0; i < fileXDim; ++i)
      {
        out << "-4 ";
      }
      out << "\n";
    }
    for (DimType y = 0; y < dims[1]; ++y)
    {
      // write leading surface voxel for this row
      if (m_AddSurfaceLayer)
      {
        out << "-5 ";
      }
      // Write the actual voxel data
      for (DimType x = 0; x < dims[0]; ++x)
      {
        if (m_FeatureIds[index] == 0)
        {
          out << "0" << " ";
        }
        else
        {
          out << m_FeatureIds[index] << " ";
        }
        ++index;
      }
      // write trailing surface voxel for this row
      if (m_AddSurfaceLayer)
      {
        out << "-6 ";
      }
      out << "\n";
    }
    // Add a trailing surface Row for this plane if needed
    if (m_AddSurfaceLayer)
    {
      for (DimType i = 0; i < fileXDim; ++i)
      {
        out << "-7 ";
      }
      out << "\n";
    }
  }

  // Add a complete layer of surface voxels
  if (m_AddSurfaceLayer)
  {
    rnIndex = 1;
    for (DimType i = 0; i < (fileXDim * fileYDim); ++i)
    {
      out << "-8 ";
      if (rnIndex == 20)
      {
        out << "\n";
        rnIndex = 0;
      }
      rnIndex++;
    }
  }
  out << "\n";
  out << "attribute \"dep\" string \"connections\"" << "\n";
  out << "\n";
  out << "object \"DREAM3D Generated\" class feature" << "\n";
  out << "component  \"positions\"    value 1" << "\n";
  out << "component  \"connections\"  value 2" << "\n";
  out << "component  \"data\"         value 3" << "\n";
  out << "" << "\n";
  out << "end" << "\n";

  file.close();
#if 0
  out.open("/tmp/m3cmesh.raw", std::ios_base::binary);
  out.write((const char*)(&dims[0]), 4);
  out.write((const char*)(&dims[1]), 4);
  out.write((const char*)(&dims[2]), 4);
  getTotalPoints = dims[0] * dims[1] * dims[2];
  int32_t d = 0;
  for(int index = 0; index < getTotalPoints; ++index)
  {
    d = featureIds[index];
    if (d == 0)
    { d = -3;}
    out.write((const char*)(&d), sizeof(d));
  }

  out.close();
#endif

  // If there is an error set this to something negative and also set a message
  notifyStatusMessage(getHumanLabel(), "Complete");
  return err;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer DxWriter::newFilterInstance(bool copyFilterParameters)
{
  DxWriter::Pointer filter = DxWriter::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DxWriter::getCompiledLibraryName()
{ return IOConstants::IOBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DxWriter::getGroupName()
{ return DREAM3D::FilterGroups::IOFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DxWriter::getSubGroupName()
{ return DREAM3D::FilterSubGroups::OutputFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DxWriter::getHumanLabel()
{ return "Write Dx File (Feature Ids)"; }
