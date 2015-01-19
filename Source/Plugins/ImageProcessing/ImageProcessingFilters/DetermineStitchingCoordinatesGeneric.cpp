/*
 * Your License or Copyright Information can go here
 */

#include "DetermineStitchingCoordinatesGeneric.h"

#include <QtCore/QString>

#include "ZeissImport/ZeissImportConstants.h"

#include "ImageProcessing/ImageProcessingFilters/util/DetermineStitching.h"


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DetermineStitchingCoordinatesGeneric::DetermineStitchingCoordinatesGeneric() :
  AbstractFilter(),
  m_AttributeMatrixName(DREAM3D::Defaults::VolumeDataContainerName, DREAM3D::Defaults::CellFeatureAttributeMatrixName, "")
/* DO NOT FORGET TO INITIALIZE ALL YOUR DREAM3D Filter Parameters HERE */
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DetermineStitchingCoordinatesGeneric::~DetermineStitchingCoordinatesGeneric()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(FilterParameter::New("Attribute Matrix Name", "AttributeMatrixName", FilterParameterWidgetType::AttributeMatrixSelectionWidget, getAttributeMatrixName(), false, ""));

  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setAttributeMatrixName(reader->readDataArrayPath("AttributeMatrixName", getAttributeMatrixName()));

  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int DetermineStitchingCoordinatesGeneric::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(AttributeMatrixName)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::dataCheck()
{
    setErrorCondition(0);
    DataArrayPath tempPath;

    QString ss;

    AttributeMatrix::Pointer am = getDataContainerArray()->getAttributeMatrix(m_AttributeMatrixName);

    if (am.get() == NULL)
    {
        setErrorCondition(-76000);
        notifyErrorMessage(getHumanLabel(), "The attribute matrix has not been selected properly", -76000);
        return;
    }

   QList<QString> names = am->getAttributeArrayNames();


    QVector<size_t> dims(1, 1);


    UInt8ArrayType::Pointer imagePtr = UInt8ArrayType::NullPointer();
    IDataArray::Pointer iDataArray = IDataArray::NullPointer();
    m_PointerList.resize(names.size());

    for(int i = 0; i < names.size(); i++)
    {
        tempPath.update(getAttributeMatrixName().getDataContainerName(), getAttributeMatrixName().getAttributeMatrixName(), names[i]);
        m_SelectedCellArrayPtr = getDataContainerArray()->getExistingPrereqArrayFromPath<DataArray<ImageProcessing::DefaultPixelType>, AbstractFilter>(this, tempPath);

        if( NULL != m_SelectedCellArrayPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
        { m_SelectedCellArray = m_SelectedCellArrayPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

        m_PointerList[i] = m_SelectedCellArray;


//        imagePtr = boost::dynamic_pointer_cast<DataArray<uint8_t> >(m_SelectedCellArrayPtr);

//        if(NULL == imagePtr)
//        {
//            setErrorCondition(-76001);
//            notifyErrorMessage(getHumanLabel(), "The data was not found", -76001);
//        }

    }


}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::preflight()
{
  // These are the REQUIRED lines of CODE to make sure the filter behaves correctly
  setInPreflight(true); // Set the fact that we are preflighting.
  emit preflightAboutToExecute(); // Emit this signal so that other widgets can do one file update
  emit updateFilterParameters(this); // Emit this signal to have the widgets push their values down to the filter
  dataCheck(); // Run our DataCheck to make sure everthing is setup correctly
  emit preflightExecuted(); // We are done preflighting this filter
  setInPreflight(false); // Inform the system this filter is NOT in preflight mode anymore.
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DetermineStitchingCoordinatesGeneric::getCompiledLibraryName()
{
  return ZeissImport::ZeissImportBaseName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DetermineStitchingCoordinatesGeneric::getGroupName()
{
  return "ImageProcessing";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DetermineStitchingCoordinatesGeneric::getHumanLabel()
{
  return "DetermineStitchingCoordinatesGeneric";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString DetermineStitchingCoordinatesGeneric::getSubGroupName()
{
  return "Misc";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::execute()
{
  int err = 0;
  dataCheck();
  if(getErrorCondition() < 0) { return; }
  setErrorCondition(0);

  VolumeDataContainer* m = getDataContainerArray()->getDataContainerAs<VolumeDataContainer>(getAttributeMatrixName().getDataContainerName());
  QString attrMatName = getAttributeMatrixName().getAttributeMatrixName();


  AttributeMatrix::Pointer attrMat = m->getAttributeMatrix(attrMatName);


  float sampleOrigin[3];
  float voxelResolution[3];

  m->getOrigin(sampleOrigin);
  m->getResolution(voxelResolution);
  QVector<size_t> udims = attrMat->getTupleDimensions();
  size_t totalPoints = attrMat->getNumTuples();

  QVector<size_t> xTileList(12);
  QVector<size_t> yTileList(12);
  QVector<size_t> newList(12);

  QVector<float> xGlobCoordsList;
  QVector<float> yGlobCoordsList;

  xTileList[0] = 0;
  xTileList[1] = 1;
  xTileList[2] = 2;
  xTileList[3] = 2;
  xTileList[4] = 1;
  xTileList[5] = 0;
  xTileList[6] = 0;
  xTileList[7] = 1;
  xTileList[8] = 2;
  xTileList[9] = 2;
  xTileList[10] = 1;
  xTileList[11] = 0;

  yTileList[0] = 0;
  yTileList[1] = 0;
  yTileList[2] = 0;
  yTileList[3] = 1;
  yTileList[4] = 1;
  yTileList[5] = 1;
  yTileList[6] = 2;
  yTileList[7] = 2;
  yTileList[8] = 2;
  yTileList[9] = 3;
  yTileList[10] = 3;
  yTileList[11] = 3;


  newList = DetermineStitching::ReturnIndexForCombOrder(xTileList, yTileList, 3, 4);




  DetermineStitching::FindGlobalOrigins(totalPoints, udims, sampleOrigin, voxelResolution, m_PointerList, xGlobCoordsList, yGlobCoordsList, xTileList, yTileList);



  /* Let the GUI know we are done with this filter */
  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer DetermineStitchingCoordinatesGeneric::newFilterInstance(bool copyFilterParameters)
{
  /*
  * write code to optionally copy the filter parameters from the current filter into the new instance
  */
  DetermineStitchingCoordinatesGeneric::Pointer filter = DetermineStitchingCoordinatesGeneric::New();
  if(true == copyFilterParameters)
  {
    /* If the filter uses all the standard Filter Parameter Widgets you can probabaly get
     * away with using this method to copy the filter parameters from the current instance
     * into the new instance
     */
    copyFilterParameterInstanceVariables(filter.get());
    /* If your filter is using a lot of custom FilterParameterWidgets @see ReadH5Ebsd then you
     * may need to copy each filter parameter explicitly plus any other instance variables that
     * are needed into the new instance. Here is some example code from ReadH5Ebsd
     */
    //    DREAM3D_COPY_INSTANCEVAR(OutputFile)
    //    DREAM3D_COPY_INSTANCEVAR(ZStartIndex)
    //    DREAM3D_COPY_INSTANCEVAR(ZEndIndex)
    //    DREAM3D_COPY_INSTANCEVAR(ZResolution)
  }
  return filter;
}

