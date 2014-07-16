/*
 * Your License or Copyright Information can go here
 */

#include "BackgroundFit.h"

#include <QtCore/QString>

#include "ImageProcessing/ImageProcessingConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
BackgroundFit::BackgroundFit() :
  AbstractFilter(),
/* DO NOT FORGET TO INITIALIZE ALL YOUR DREAM3D Filter Parameters HERE */
m_SelectedCellArrayPath("", "", ""),
m_SelectedCellArrayArrayName(""),
m_SelectedCellArray(NULL)
{
setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
BackgroundFit::~BackgroundFit()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void BackgroundFit::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(FilterParameter::New("Array to Process", "SelectedCellArrayPath", FilterParameterWidgetType::DataArraySelectionWidget, getSelectedCellArrayPath(), false, ""));
  /* There are several types of FilterParameter classes to choose from and several
  * options for each class type. The programmer can put the entire invocation into
  * a single line if they want. For example:
  *
  *   parameters.push_back(FilterParameter::New("Reference Direction", "ReferenceDir", FilterParameterWidgetType::FloatVec3Widget, getReferenceDir(), false));
  * or the programmer can create a FilterParameter like usual C++ codes:
  * {
  *  FilterParameter::Pointer parameter = FilterParameter::New();
  *  parameter->setHumanLabel("Eulers Array");
  *  parameter->setPropertyName("CellEulerAnglesArrayName");
  *  parameter->setWidgetType(FilterParameterWidgetType::SingleArraySelectionWidget);
  *  parameter->setUnits("");
  *  parameters.push_back(parameter);
  * }
  */
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void BackgroundFit::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setSelectedCellArrayPath( reader->readDataArrayPath( "SelectedCellArrayPath", getSelectedCellArrayPath() ) );
  /*
   Place code in here that will read the parameters from a file
   setOutputFile( reader->readValue("OutputFile", getOutputFile() ) );
   */
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int BackgroundFit::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(SelectedCellArrayPath)
  /* Place code that will write the inputs values into a file. reference the AbstractFilterParametersWriter class for the proper API to use. */
  /*  writer->writeValue("OutputFile", getOutputFile() ); */
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void BackgroundFit::dataCheck()
{
  setErrorCondition(0);
  DataArrayPath tempPath;

  QVector<size_t> dims(1, 1);
  m_SelectedCellArrayPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<ImageProcessing::DefaultPixelType>, AbstractFilter>(this, getSelectedCellArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_SelectedCellArrayPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_SelectedCellArray = m_SelectedCellArrayPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */


  /* Example code for preflighting looking for a valid string for the output file
   * but not necessarily the fact that the file exists: Example code to make sure
   * we have something in a string before proceeding.*/
  /*
  if (m_OutputFile.empty() == true)
  {
    QString ss = QObject::tr("Output file name was not set").arg(getHumanLabel());
    setErrorCondition(-1);
    notifyErrorMessage(getHumanLabel(), ss, -1);
    return;
  }
  * We can also check for the availability of REQUIRED ARRAYS:
  * QVector<size_t> dims(1, 1);
  * // Assigns the shared_ptr<> to an instance variable that is a weak_ptr<>
  * m_CellPhasesPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter>(this, getCellPhasesArrayPath(), dims);
  *  // Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object
  * if( NULL != m_CellPhasesPtr.lock().get() )
  * {
  *   // Now assign the raw pointer to data from the DataArray<T> object
  *   m_CellPhases = m_CellPhasesPtr.lock()->getPointer(0);
  * }
  *
  * We can also CREATE a new array to dump new data into
  *   tempPath.update(m_CellEulerAnglesArrayPath.getDataContainerName(), m_CellEulerAnglesArrayPath.getAttributeMatrixName(), getCellIPFColorsArrayName() );
  * // Assigns the shared_ptr<> to an instance variable that is a weak_ptr<>
  * m_CellIPFColorsPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<uint8_t>, AbstractFilter, uint8_t>(this, tempPath, 0, dims);
  * // Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object
  * if( NULL != m_CellIPFColorsPtr.lock().get() )
  * {
  * // Now assign the raw pointer to data from the DataArray<T> object
  * m_CellIPFColors = m_CellIPFColorsPtr.lock()->getPointer(0);
  * }
  */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void BackgroundFit::preflight()
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
const QString BackgroundFit::getCompiledLibraryName()
{
  return ImageProcessing::ImageProcessingBaseName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString BackgroundFit::getGroupName()
{
  return "ImageProcessing";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString BackgroundFit::getHumanLabel()
{
  return "BackgroundFit";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString BackgroundFit::getSubGroupName()
{
  return "Misc";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void BackgroundFit::execute()
{
  int err = 0;
  dataCheck();
  if(getErrorCondition() < 0) { return; }
  setErrorCondition(0);

  /* If some error occurs this code snippet can report the error up the call chain*/
  if (err < 0)
  {
    QString ss = QObject::tr("Some error message");
    setErrorCondition(-99999999);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
    return;
  }

  VolumeDataContainer* m = getDataContainerArray()->getDataContainerAs<VolumeDataContainer>(getSelectedCellArrayPath().getDataContainerName());
  QString attrMatName = getSelectedCellArrayPath().getAttributeMatrixName();

  size_t udims[3] = {0, 0, 0};
  m->getDimensions(udims);

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








  /* Let the GUI know we are done with this filter */
  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer BackgroundFit::newFilterInstance(bool copyFilterParameters)
{
  /*
  * write code to optionally copy the filter parameters from the current filter into the new instance
  */
  BackgroundFit::Pointer filter = BackgroundFit::New();
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

