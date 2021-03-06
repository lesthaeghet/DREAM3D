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


#include "ImportImageStack.h"

#include <QtGui/QImageReader>

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/FileListInfoFilterParameter.h"
#include "DREAM3DLib/FilterParameters/SeparatorFilterParameter.h"
#include "DREAM3DLib/Utilities/FilePathGenerator.h"

#include "ImageIO/ImageIOConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ImportImageStack::ImportImageStack() :
  AbstractFilter(),
  m_DataContainerName(DREAM3D::Defaults::DataContainerName),
  m_CellAttributeMatrixName(DREAM3D::Defaults::CellAttributeMatrixName),
  m_ImageDataArrayName(DREAM3D::CellData::ImageData)
{
  m_Origin.x = 0.0f;
  m_Origin.y = 0.0f;
  m_Origin.z = 0.0f;

  m_Resolution.x = 1.0f;
  m_Resolution.y = 1.0f;
  m_Resolution.z = 1.0f;

  m_InputFileListInfo.FileExtension = QString("tif");
  m_InputFileListInfo.StartIndex = 0;
  m_InputFileListInfo.EndIndex = 0;
  m_InputFileListInfo.PaddingDigits = 0;

  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ImportImageStack::~ImportImageStack()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ImportImageStack::setupFilterParameters()
{
  QVector<FilterParameter::Pointer> parameters;
  parameters.push_back(FileListInfoFilterParameter::New("Input File List", "InputFileListInfo", getInputFileListInfo(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Origin", "Origin", FilterParameterWidgetType::FloatVec3Widget, getOrigin(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Resolution", "Resolution", FilterParameterWidgetType::FloatVec3Widget, getResolution(), FilterParameter::Parameter));
  parameters.push_back(FilterParameter::New("Data Container Name", "DataContainerName", FilterParameterWidgetType::StringWidget, getDataContainerName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Cell Attribute Matrix Name", "CellAttributeMatrixName", FilterParameterWidgetType::StringWidget, getCellAttributeMatrixName(), FilterParameter::CreatedArray, ""));
  parameters.push_back(FilterParameter::New("Image Data", "ImageDataArrayName", FilterParameterWidgetType::StringWidget, getImageDataArrayName(), FilterParameter::CreatedArray, ""));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ImportImageStack::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setDataContainerName(reader->readString("DataContainerName", getDataContainerName() ) );
  setCellAttributeMatrixName(reader->readString("CellAttributeMatrixName", getCellAttributeMatrixName() ) );
  setImageDataArrayName(reader->readString("ImageDataArrayName", getImageDataArrayName() ) );
  setInputFileListInfo( reader->readFileListInfo("InputFileListInfo", getInputFileListInfo() ) );
  setOrigin( reader->readFloatVec3("Origin", getOrigin()) );
  setResolution( reader->readFloatVec3("Resolution", getResolution()) );
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int ImportImageStack::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  DREAM3D_FILTER_WRITE_PARAMETER(DataContainerName)
  DREAM3D_FILTER_WRITE_PARAMETER(CellAttributeMatrixName)
  DREAM3D_FILTER_WRITE_PARAMETER(ImageDataArrayName)
  DREAM3D_FILTER_WRITE_PARAMETER(InputFileListInfo)
  DREAM3D_FILTER_WRITE_PARAMETER(Origin)
  DREAM3D_FILTER_WRITE_PARAMETER(Resolution)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ImportImageStack::dataCheck()
{
  setErrorCondition(0);

  DataArrayPath tempPath;
  QString ss;

  if (m_InputFileListInfo.InputPath.isEmpty() == true)
  {
    ss = QObject::tr("The input directory must be set");
    notifyErrorMessage(getHumanLabel(), ss, -13);
    setErrorCondition(-13);
  }

  DataContainer::Pointer m = getDataContainerArray()->createNonPrereqDataContainer<AbstractFilter>(this, getDataContainerName());
  if(getErrorCondition() < 0 || NULL == m.get()) { return; }

  ImageGeom::Pointer image = ImageGeom::CreateGeometry(DREAM3D::Geometry::ImageGeometry);
  m->setGeometry(image);

  bool hasMissingFiles = false;
  bool orderAscending = false;

  if (m_InputFileListInfo.Ordering == 0) { orderAscending = true; }
  else if (m_InputFileListInfo.Ordering == 1) { orderAscending = false; }

  // Now generate all the file names the user is asking for and populate the table
  QVector<QString> fileList = FilePathGenerator::GenerateFileList(m_InputFileListInfo.StartIndex,
                              m_InputFileListInfo.EndIndex, hasMissingFiles, orderAscending,
                              m_InputFileListInfo.InputPath, m_InputFileListInfo.FilePrefix,
                              m_InputFileListInfo.FileSuffix, m_InputFileListInfo.FileExtension,
                              m_InputFileListInfo.PaddingDigits);
  if (fileList.size() == 0)
  {
    QString ss = QObject::tr("No files have been selected for import");
    setErrorCondition(-11);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }
  else
  {
    // If we have RGB or RGBA Images then we are going to have to change things a bit.
    // We should read the file and see what we have? Of course Qt is going to read it up into
    // an RGB array by default
    int32_t err = 0;
    QImageReader reader((fileList[0]));
    QSize imageDims = reader.size();
    int64_t dims[3] = { imageDims.width(), imageDims.height(), fileList.size() };
    /* Sanity check what we are trying to load to make sure it can fit in our address space.
     * Note that this does not guarantee the user has enough left, just that the
     * size of the volume can fit in the address space of the program
     */
#if   (CMP_SIZEOF_SSIZE_T==4)
    int64_t max = std::numeric_limits<size_t>::max();
#else
    int64_t max = std::numeric_limits<int64_t>::max();
#endif
    if (dims[0] * dims[1] * dims[2] > max)
    {
      err = -1;
      QString ss = QObject::tr("The total number of elements '%1' is greater than this program can hold").arg((dims[0] * dims[1] * dims[2]));
      setErrorCondition(err);
      notifyErrorMessage(getHumanLabel(), ss, err);
    }

    if (dims[0] > max || dims[1] > max || dims[2] > max)
    {
      err = -1;
      QString ss = QObject::tr("One of the dimensions is greater than the max index for this sysem"
                               " dim[0]=%1  dim[1]=%2  dim[2]=%3").arg(dims[0]).arg(dims[1]).arg(dims[2]);
      setErrorCondition(err);
      notifyErrorMessage(getHumanLabel(), ss, err);
    }
    /* ************ End Sanity Check *************************** */

    m->getGeometryAs<ImageGeom>()->setDimensions(static_cast<size_t>(dims[0]), static_cast<size_t>(dims[1]), static_cast<size_t>(dims[2]));
    m->getGeometryAs<ImageGeom>()->setResolution(m_Resolution.x, m_Resolution.y, m_Resolution.z);
    m->getGeometryAs<ImageGeom>()->setOrigin(m_Origin.x, m_Origin.y, m_Origin.z);

    QVector<size_t> tDims(3, 0);
    for (int32_t i = 0; i < 3; i++)
    {
      tDims[i] = dims[i];
    }
    m->createNonPrereqAttributeMatrix<AbstractFilter>(this, getCellAttributeMatrixName(), tDims, DREAM3D::AttributeMatrixType::Cell);

    int32_t pixelBytes = 0;

    if (reader.imageFormat() == QImage::Format_Indexed8)
    {
      pixelBytes = 1;
    }
    else if (reader.imageFormat() == QImage::Format_RGB32 || reader.imageFormat() == QImage::Format_ARGB32)
    {
      pixelBytes = 4;
    }
    else {
      ss = QObject::tr("Image format is of unsupported type. Imported images must be either grayscale, RGB, or ARGB");
      setErrorCondition(-1);
      notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    }

    QVector<size_t> cDims(1, pixelBytes);
    // This would be for a gray scale image
    tempPath.update(getDataContainerName(), getCellAttributeMatrixName(), getImageDataArrayName() );
    m_ImageDataPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<uint8_t>, AbstractFilter, uint8_t>(this, tempPath, 0, cDims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_ImageDataPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    { m_ImageData = m_ImageDataPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  }
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ImportImageStack::preflight()
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
void ImportImageStack::execute()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getDataContainerName());

  m->getGeometryAs<ImageGeom>()->setResolution(m_Resolution.x, m_Resolution.y, m_Resolution.z);
  m->getGeometryAs<ImageGeom>()->setOrigin(m_Origin.x, m_Origin.y, m_Origin.z);

  UInt8ArrayType::Pointer data = UInt8ArrayType::NullPointer();

  uint8_t* imagePtr = NULL;

  int32_t pixelBytes = 0;
  int32_t totalPixels = 0;
  int32_t height = 0;
  int32_t width = 0;

  int64_t z = m_InputFileListInfo.StartIndex;
  int64_t zSpot;

  bool hasMissingFiles = false;
  bool orderAscending = false;

  if (m_InputFileListInfo.Ordering == 0) { orderAscending = true; }
  else if (m_InputFileListInfo.Ordering == 1) { orderAscending = false; }

  // Now generate all the file names the user is asking for and populate the table
  QVector<QString> fileList = FilePathGenerator::GenerateFileList(m_InputFileListInfo.StartIndex,
                              m_InputFileListInfo.EndIndex, hasMissingFiles, orderAscending,
                              m_InputFileListInfo.InputPath, m_InputFileListInfo.FilePrefix,
                              m_InputFileListInfo.FileSuffix, m_InputFileListInfo.FileExtension,
                              m_InputFileListInfo.PaddingDigits);
  if (fileList.size() == 0)
  {
    QString ss = QObject::tr("No files have been selected for import");
    setErrorCondition(-11);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  for (QVector<QString>::iterator filepath = fileList.begin(); filepath != fileList.end(); ++filepath)
  {
    QString imageFName = *filepath;
    QString ss = QObject::tr("Importing file %1").arg(imageFName);
    notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss);

    QImage image(imageFName);
    if (image.isNull() == true)
    {
      setErrorCondition(-14000);
      notifyErrorMessage(getHumanLabel(), "Failed to load image file", getErrorCondition());
      return;
    }
    height = image.height();
    width = image.width();
    totalPixels = width * height;
    // This is the first image so we need to create our block of data to store the data
    if (z == m_InputFileListInfo.StartIndex)
    {
      m->getGeometryAs<ImageGeom>()->setDimensions(width, height, fileList.size());
      if (image.format() == QImage::Format_Indexed8)
      {
        pixelBytes = 1;
      }
      else if (image.format() == QImage::Format_RGB32 || image.format() == QImage::Format_ARGB32)
      {
        pixelBytes = 4;
      }
      else {
        ss = QObject::tr("Image format is of unsupported type. Imported images must be either grayscale, RGB, or ARGB");
        setErrorCondition(-1);
        notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
        return;
      }
      QVector<size_t> compDims(1, pixelBytes);

      data = UInt8ArrayType::CreateArray(size_t(fileList.size()) * height * width, compDims, m_ImageDataArrayName);
    }

    // Get the current position in the array to copy the image into
    zSpot = (z -  m_InputFileListInfo.StartIndex);
    for (int32_t i = 0; i < height; ++i)
    {
      imagePtr = data->getPointer( (zSpot) * totalPixels * pixelBytes + i * (width * pixelBytes));
      uint8_t* source = image.scanLine(i);
      ::memcpy(imagePtr, source, width * pixelBytes);
    }

    ++z;
    if (getCancel() == true) { return; }
  }

  m->getAttributeMatrix(getCellAttributeMatrixName())->addAttributeArray(data->getName(), data);

  /* Let the GUI know we are done with this filter */
  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer ImportImageStack::newFilterInstance(bool copyFilterParameters)
{
  ImportImageStack::Pointer filter = ImportImageStack::New();
  if(true == copyFilterParameters)
  {
    filter->setFilterParameters(getFilterParameters() );
    // We are going to hand copy all of the parameters because the other way of copying the parameters are going to
    // miss some of them because we are not enumerating all of them.
    DREAM3D_COPY_INSTANCEVAR(DataContainerName)
    DREAM3D_COPY_INSTANCEVAR(CellAttributeMatrixName)
    DREAM3D_COPY_INSTANCEVAR(Resolution)
    DREAM3D_COPY_INSTANCEVAR(Origin)
#if 0
    DREAM3D_COPY_INSTANCEVAR(ZStartIndex)
    DREAM3D_COPY_INSTANCEVAR(ZEndIndex)
    DREAM3D_COPY_INSTANCEVAR(InputPath)
    DREAM3D_COPY_INSTANCEVAR(FilePrefix)
    DREAM3D_COPY_INSTANCEVAR(FileSuffix)
    DREAM3D_COPY_INSTANCEVAR(FileExtension)
    DREAM3D_COPY_INSTANCEVAR(PaddingDigits)
    DREAM3D_COPY_INSTANCEVAR(RefFrameZDir)
#endif
    DREAM3D_COPY_INSTANCEVAR(InputFileListInfo)
    DREAM3D_COPY_INSTANCEVAR(ImageStack)
    DREAM3D_COPY_INSTANCEVAR(ImageDataArrayName)
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString ImportImageStack::getCompiledLibraryName()
{ return ImageIOConstants::ImageIOBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString ImportImageStack::getGroupName()
{ return DREAM3D::FilterGroups::IOFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString ImportImageStack::getSubGroupName()
{ return DREAM3D::FilterSubGroups::InputFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString ImportImageStack::getHumanLabel()
{ return "Import Images (3D Stack)"; }
