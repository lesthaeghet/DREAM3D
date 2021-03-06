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

#include "VASPReader.h"

#include <QtCore/QtDebug>
#include <fstream>

#include <QtCore/QFileInfo>

#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/FileSystemFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"
#include "DREAM3DLib/Math/MatrixMath.h"

#include "IO/IOConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
VASPReader::VASPReader() :
  FileReader(),
  m_VertexDataContainerName(DREAM3D::Defaults::DataContainerName),
  m_VertexAttributeMatrixName(DREAM3D::Defaults::VertexAttributeMatrixName),
  m_InputFile(""),
  m_AtomVelocitiesArrayName(DREAM3D::VertexData::AtomVelocities),
  m_AtomTypesArrayName(DREAM3D::VertexData::AtomTypes),
  m_AtomVelocities(NULL),
  m_AtomTypes(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
VASPReader::~VASPReader()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void VASPReader::setupFilterParameters()
{
  FilterParameterVector parameters;

  parameters.push_back(FileSystemFilterParameter::New("Input File", "InputFile", FilterParameterWidgetType::InputFileWidget, getInputFile(), FilterParameter::Parameter, "", "*"));

  parameters.push_back(FilterParameter::New("Vertex Data Container", "VertexDataContainerName", FilterParameterWidgetType::StringWidget, getVertexDataContainerName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Vertex Attribute Matrix", "VertexAttributeMatrixName", FilterParameterWidgetType::StringWidget, getVertexAttributeMatrixName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("AtomVelocities", "AtomVelocitiesArrayName", FilterParameterWidgetType::StringWidget, getAtomVelocitiesArrayName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("AtomTypes", "AtomTypesArrayName", FilterParameterWidgetType::StringWidget, getAtomTypesArrayName(), FilterParameter::CreatedArray, ""));

  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
void VASPReader::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setVertexDataContainerName(reader->readString("VertexDataContainerName", getVertexDataContainerName() ) );
  setVertexAttributeMatrixName(reader->readString("VertexAttributeMatrixName", getVertexAttributeMatrixName() ) );
  setAtomTypesArrayName(reader->readString("AtomTypesArrayName", getAtomTypesArrayName() ) );
  setAtomVelocitiesArrayName(reader->readString("AtomVelocitiesArrayName", getAtomVelocitiesArrayName() ) );
  setInputFile( reader->readString( "InputFile", getInputFile() ) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int VASPReader::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(VertexDataContainerName)
  writer->writeValue("VertexAttributeMatrixName", getVertexAttributeMatrixName() ) ;
  DREAM3D_FILTER_WRITE_PARAMETER(AtomTypesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(AtomVelocitiesArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(InputFile)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void VASPReader::updateVertexInstancePointers()
{
  setErrorCondition(0);

  if( NULL != m_AtomVelocitiesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_AtomVelocities = m_AtomVelocitiesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  if( NULL != m_AtomTypesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_AtomTypes = m_AtomTypesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void VASPReader::dataCheck()
{
  DataArrayPath tempPath;
  setErrorCondition(0);
  DataContainer::Pointer m = getDataContainerArray()->createNonPrereqDataContainer<AbstractFilter>(this, getVertexDataContainerName());
  if(getErrorCondition() < 0) { return; }
  QVector<size_t> tDims (1, 0);
  AttributeMatrix::Pointer vertexAttrMat = m->createNonPrereqAttributeMatrix<AbstractFilter>(this, getVertexAttributeMatrixName(), tDims, DREAM3D::AttributeMatrixType::Vertex);
  if(getErrorCondition() < 0 || NULL == vertexAttrMat.get()) { return; }

  VertexGeom::Pointer vertex = VertexGeom::CreateGeometry(0, DREAM3D::Geometry::VertexGeometry);
  m->setGeometry(vertex);

  QFileInfo fi(getInputFile());

  if (getInputFile().isEmpty() == true)
  {
    QString ss = QObject::tr("%1 needs the Input File Set and it was not.").arg(ClassName());
    setErrorCondition(-387);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }
  else if (fi.exists() == false)
  {
    QString ss = QObject::tr("The input file does not exist.");
    setErrorCondition(-388);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  QVector<size_t> dims(1, 3);
  tempPath.update(getVertexDataContainerName(), getVertexAttributeMatrixName(), getAtomVelocitiesArrayName() );
  m_AtomVelocitiesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this,  tempPath, 0.0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_AtomVelocitiesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_AtomVelocities = m_AtomVelocitiesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  dims[0] = 1;
  tempPath.update(getVertexDataContainerName(), getVertexAttributeMatrixName(), getAtomTypesArrayName() );
  m_AtomTypesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter, int32_t>(this,  tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_AtomTypesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_AtomTypes = m_AtomTypesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

  if (m_InStream.isOpen() == true)
  {
    m_InStream.close();
  }

  if (getInputFile().isEmpty() == false && fi.exists() == true)
  {
    // We need to read the header of the input file to get the dimensions
    m_InStream.setFileName(getInputFile());
    if (!m_InStream.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      QString ss = QObject::tr("VASPReader Input file could not be opened: %1").arg(getInputFile());
      setErrorCondition(-100);
      notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
      return;
    }

    int error = readHeader();
    m_InStream.close();
    if (error < 0)
    {
      setErrorCondition(error);
      QString ss = QObject::tr("Error occurred trying to parse the dimensions from the input file. Is the input file a VASP file?");
      notifyErrorMessage(getHumanLabel(), ss, -11000);
    }
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void VASPReader::preflight()
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
void VASPReader::execute()
{
  int err = 0;

  dataCheck();
  if(getErrorCondition() < 0) { return; }

  m_InStream.setFileName(getInputFile());
  if (!m_InStream.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    QString ss = QObject::tr("VASPReader Input file could not be opened: %1").arg(getInputFile());
    setErrorCondition(-100);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return;
  }

  err = readHeader();
  if(err < 0)
  {
    m_InStream.close();
    return;
  }
  err = readFile();
  m_InStream.close();
  if(err < 0)
  {
    return;
  }

}


//-----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int VASPReader::readHeader()
{
  QString ss;

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getVertexDataContainerName());
  AttributeMatrix::Pointer vertexAttrMat = m->getAttributeMatrix(getVertexAttributeMatrixName());

  int error = 0;

  QByteArray buf;
  QList<QByteArray> tokens; /* vector to store the split data */

  bool ok = false;
  // Process the header information
//  bool done = false;
  //read Title line
  buf = m_InStream.readLine();
  //read Lattice Constant line
  buf = m_InStream.readLine();
  buf = buf.trimmed();
  tokens = buf.split(' ');
  latticeConstant = tokens[0].toFloat(&ok);
  //read Lattice Vector lines
  buf = m_InStream.readLine();
  buf = buf.trimmed();
  buf = buf.simplified();
  tokens = buf.split(' ');
  latticeVectors[0][0] = tokens[0].toFloat(&ok);
  latticeVectors[0][1] = tokens[1].toFloat(&ok);
  latticeVectors[0][2] = tokens[2].toFloat(&ok);
  buf = m_InStream.readLine();
  buf = buf.trimmed();
  buf = buf.simplified();
  tokens = buf.split(' ');
  latticeVectors[1][0] = tokens[0].toFloat(&ok);
  latticeVectors[1][1] = tokens[1].toFloat(&ok);
  latticeVectors[1][2] = tokens[2].toFloat(&ok);
  buf = m_InStream.readLine();
  buf = buf.trimmed();
  buf = buf.simplified();
  tokens = buf.split(' ');
  latticeVectors[2][0] = tokens[0].toFloat(&ok);
  latticeVectors[2][1] = tokens[1].toFloat(&ok);
  latticeVectors[2][2] = tokens[2].toFloat(&ok);
  //read Atom Type line
  buf = m_InStream.readLine();
  //read Atom Number line
  buf = m_InStream.readLine();
  buf = buf.trimmed();
  buf = buf.simplified();
  tokens = buf.split(' ');
  atomNumbers.resize(tokens.size());
  totalAtoms = 0;
  for(int i = 0; i < tokens.size(); i++)
  {
    atomNumbers[i] = tokens[i].toInt(&ok, 10);
    totalAtoms += tokens[i].toInt(&ok, 10);
  }

  VertexGeom::Pointer vertices = VertexGeom::CreateGeometry(totalAtoms, DREAM3D::VertexData::SurfaceMeshNodes);
  m->setGeometry(vertices);

  QVector<size_t> tDims(1, totalAtoms);
  vertexAttrMat->resizeAttributeArrays(tDims);
  updateVertexInstancePointers();

  return error;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int VASPReader::readFile()
{
  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getVertexDataContainerName());

  QByteArray buf;
  QList<QByteArray> tokens; /* vector to store the split data */

  VertexGeom::Pointer verticesPtr = m->getGeometryAs<VertexGeom>();
  float* vertex = verticesPtr->getVertexPointer(0);

  //read the blank line
  buf = m_InStream.readLine();
  bool ok = false;

  MatrixMath::Multiply3x3withConstant(latticeVectors, latticeConstant);

  float pos[3];
  float posMod[3];
  int index = 0;
  for(int i = 0; i < atomNumbers.size(); i++)
  {
    for(int j = 0; j < atomNumbers[i]; j++)
    {
      buf = m_InStream.readLine();
      buf = buf.trimmed();
      buf = buf.simplified();
      tokens = buf.split(' ');
      pos[0] = tokens[0].toFloat(&ok);
      pos[1] = tokens[1].toFloat(&ok);
      pos[2] = tokens[2].toFloat(&ok);
      MatrixMath::Multiply3x3with3x1(latticeVectors, pos, posMod);
      vertex[index*3] = posMod[0];
      vertex[index*3+1] = posMod[1];
      vertex[index*3+2] = posMod[2];
      m_AtomTypes[index] = i;
      index++;
    }
  }
  //read the blank line
  buf = m_InStream.readLine();

  float vel[3];
  float velMod[3];

  index = 0;
  for(int i = 0; i < atomNumbers.size(); i++)
  {
    for(int j = 0; j < atomNumbers[i]; j++)
    {
      buf = m_InStream.readLine();
      buf = buf.trimmed();
      buf = buf.simplified();
      tokens = buf.split(' ');
      vel[0] = tokens[0].toFloat(&ok);
      vel[1] = tokens[1].toFloat(&ok);
      vel[2] = tokens[2].toFloat(&ok);
      MatrixMath::Multiply3x3with3x1(latticeVectors, vel, velMod);
      m_AtomVelocities[3 * index + 0] = velMod[0];
      m_AtomVelocities[3 * index + 1] = velMod[1];
      m_AtomVelocities[3 * index + 2] = velMod[2];
      index++;
    }
  }

  tokens.clear();
  m_InStream.close();

  notifyStatusMessage(getHumanLabel(), "Complete");
  return 0;
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer VASPReader::newFilterInstance(bool copyFilterParameters)
{
  VASPReader::Pointer filter = VASPReader::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString VASPReader::getCompiledLibraryName()
{ return IOConstants::IOBaseName; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString VASPReader::getGroupName()
{ return DREAM3D::FilterGroups::IOFilters; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString VASPReader::getSubGroupName()
{ return DREAM3D::FilterSubGroups::InputFilters; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString VASPReader::getHumanLabel()
{ return "Read VASP File"; }

