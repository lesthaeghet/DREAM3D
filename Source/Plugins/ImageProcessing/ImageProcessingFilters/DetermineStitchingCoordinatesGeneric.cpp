/*
 * Your License or Copyright Information can go here
 */

#include "DetermineStitchingCoordinatesGeneric.h"

#include <QtCore/QString>

#include "ZeissImport/ZeissImportConstants.h"

#include "ImageProcessing/ImageProcessingFilters/util/DetermineStitching.h"

#include "DREAM3DLib/DataArrays/StringDataArray.hpp"


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DetermineStitchingCoordinatesGeneric::DetermineStitchingCoordinatesGeneric() :
    AbstractFilter(),
    m_AttributeMatrixName(DREAM3D::Defaults::VolumeDataContainerName, DREAM3D::Defaults::CellFeatureAttributeMatrixName, ""),
    m_MetaDataAttributeMatrixName("")
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
    QStringList linkedProps;

    linkedProps << "MetaDataAttributeMatrixName";

    parameters.push_back(LinkedBooleanFilterParameter::New("Use Zeiss Meta Data", "UseZeissMetaData", getUseZeissMetaData(), linkedProps, false));

    parameters.push_back(FilterParameter::New("Attribute Matrix Name", "AttributeMatrixName", FilterParameterWidgetType::AttributeMatrixSelectionWidget, getAttributeMatrixName(), false, ""));
    parameters.push_back(FilterParameter::New("Meta Data for Attribute Matrix", "MetaDataAttributeMatrixName", FilterParameterWidgetType::AttributeMatrixSelectionWidget, getMetaDataAttributeMatrixName(), false, ""));
    linkedProps.clear();

    parameters.push_back(FilterParameter::New("Calculated Tile Info Attribute Matrix Name", "TileCalculatedInfoAttributeMatrixName", FilterParameterWidgetType::StringWidget, getTileCalculatedInfoAttributeMatrixName(), true, ""));
    parameters.push_back(FilterParameter::New("Stiched Coordinates", "StitchedCoordinatesArrayName", FilterParameterWidgetType::StringWidget, getStitchedCoordinatesArrayName(), true, ""));
    parameters.push_back(FilterParameter::New("Stitched Coordinates Data Array Name List", "DataArrayNameForStitchedCoordinatesArrayName", FilterParameterWidgetType::StringWidget, getDataArrayNameForStitchedCoordinatesArrayName(), true, ""));

    setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DetermineStitchingCoordinatesGeneric::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
    reader->openFilterGroup(this, index);
    setAttributeMatrixName(reader->readDataArrayPath("AttributeMatrixName", getAttributeMatrixName()));
    setMetaDataAttributeMatrixName(reader->readDataArrayPath("MetaDataAttributeMatrixName", getMetaDataAttributeMatrixName()));
    setTileCalculatedInfoAttributeMatrixName(reader->readString("TileCalculatedInfoAttributeMatrixName", getTileCalculatedInfoAttributeMatrixName()));
    setStitchedCoordinatesArrayName(reader->readString("StitchedCoordinatesArrayName", getStitchedCoordinatesArrayName()));
    setDataArrayNameForStitchedCoordinatesArrayName(reader->readString("DataArrayNamesForStitchedCoordinates", getDataArrayNameForStitchedCoordinatesArrayName()));
    reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int DetermineStitchingCoordinatesGeneric::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
    writer->openFilterGroup(this, index);
    DREAM3D_FILTER_WRITE_PARAMETER(AttributeMatrixName)
    DREAM3D_FILTER_WRITE_PARAMETER(MetaDataAttributeMatrixName)
    DREAM3D_FILTER_WRITE_PARAMETER(TileCalculatedInfoAttributeMatrixName)
    DREAM3D_FILTER_WRITE_PARAMETER(StitchedCoordinatesArrayName)
    DREAM3D_FILTER_WRITE_PARAMETER(DataArrayNameForStitchedCoordinatesArrayName)
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



    m_PointerList.resize(names.size());

    for(int i = 0; i < names.size(); i++)
    {
        tempPath.update(getAttributeMatrixName().getDataContainerName(), getAttributeMatrixName().getAttributeMatrixName(), names[i]);
        m_SelectedCellArrayPtr = getDataContainerArray()->getExistingPrereqArrayFromPath<DataArray<ImageProcessing::DefaultPixelType>, AbstractFilter>(this, tempPath);

        if( NULL != m_SelectedCellArrayPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
        { m_SelectedCellArray = m_SelectedCellArrayPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

        m_PointerList[i] = m_SelectedCellArray;

    }

    if(m_UseZeissMetaData == true)
    {
        AttributeMatrix::Pointer MetaDataAm = getDataContainerArray()->getAttributeMatrix(m_MetaDataAttributeMatrixName);
        if(NULL == MetaDataAm.get())
        {
            notifyErrorMessage(getHumanLabel(), "The Attribute Matrix was not found", -76001);
            return;
        }
        //        QString temp = "_META_DATA";
        bool a = getMetaDataAttributeMatrixName().getAttributeMatrixName().contains("_META_DATA");
        if (a == false)
        {
            notifyErrorMessage(getHumanLabel(), "The Attribute Matrix does not contain the Zeiss Meta Data", -76002);
            return;
        }

    }

    VolumeDataContainer* m = getDataContainerArray()->getDataContainerAs<VolumeDataContainer>(getAttributeMatrixName().getDataContainerName());
    if(getErrorCondition() < 0 || NULL == m) { return; }

    QVector<size_t> tDims(1, m_PointerList.size());
    AttributeMatrix::Pointer AttrMat = m->createNonPrereqAttributeMatrix<AbstractFilter>(this, getTileCalculatedInfoAttributeMatrixName(), tDims, DREAM3D::AttributeMatrixType::CellFeature);
    if(getErrorCondition() < 0) { return; }

    dims[0]=2;

    tempPath.update(getAttributeMatrixName().getDataContainerName(), getTileCalculatedInfoAttributeMatrixName(), getStitchedCoordinatesArrayName());
    m_StitchedCoordinatesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this,  tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_StitchedCoordinatesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    { m_StitchedCoordinates = m_StitchedCoordinatesPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

    dims[0]=1;



    tempPath.update(getAttributeMatrixName().getDataContainerName(), getTileCalculatedInfoAttributeMatrixName(), getDataArrayNameForStitchedCoordinatesArrayName() );
    m_DataArrayNamesForStitchedCoordinatesPtr = getDataContainerArray()->createNonPrereqArrayFromPath<StringDataArray, AbstractFilter, std::string>(this,  tempPath, "0", dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
    if( NULL != m_DataArrayNamesForStitchedCoordinatesPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
    {
     //   m_DataArrayNamesForStitchedCoordinates = m_DataArrayNamesForStitchedCoordinatesPtr.lock()->getPointer(0);  /* Now assign the raw pointer to data from the DataArray<T> object */
    }

    return;
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

    QString XTileIndexName = "ImageIndexU";
    QString YTileIndexName = "ImageIndexV";
    QString XGlobalIndexName = "StagePositionX";
    QString YGlobalIndexName = "StagePositionY";
    QString XScale = "ScaleFactorForX";
    QString YScale = "ScaleFactorForY";

    QVector<size_t> xTileList(m_PointerList.size());
    QVector<size_t> yTileList(m_PointerList.size());
    QVector<float> xGlobCoordsList(m_PointerList.size());
    QVector<float> yGlobCoordsList(m_PointerList.size());

    AttributeMatrix::Pointer attrMat = m->getAttributeMatrix(attrMatName);

    xTileList = extractTileIndices(XTileIndexName);
    yTileList = extractTileIndices(YTileIndexName);

    xGlobCoordsList = extractGlobalIndices(XGlobalIndexName, XScale);
    yGlobCoordsList = extractGlobalIndices(YGlobalIndexName, YScale);

    float sampleOrigin[3];
    float voxelResolution[3];

    m->getOrigin(sampleOrigin);
    m->getResolution(voxelResolution);
    QVector<size_t> udims = attrMat->getTupleDimensions();
    size_t totalPoints = attrMat->getNumTuples();



    FloatArrayType::Pointer temp = DetermineStitching::FindGlobalOrigins(totalPoints, udims, sampleOrigin, voxelResolution, m_PointerList, xGlobCoordsList, yGlobCoordsList, xTileList, yTileList, this);


    float* src = temp->getPointer(0);
    float* dest = m_StitchedCoordinatesPtr.lock()->getPointer(0);
    size_t totalBytes = (m_StitchedCoordinatesPtr.lock()->getNumberOfTuples() * m_StitchedCoordinatesPtr.lock()->getNumberOfComponents() * sizeof(float));
    ::memcpy(dest, src, totalBytes);

    StringDataArray::Pointer arrayNames = m_DataArrayNamesForStitchedCoordinatesPtr.lock();
    arrayNames->resize(m_PointerList.size());

    //Create another data array with the list of names of the images in the same order as the returned stitched coordinates
    QList<QString> names = attrMat->getAttributeArrayNames();
    for(size_t i = 0; i < names.size(); i++ )
    {
        arrayNames->setValue(i, names[i]);
    }

    /* Let the GUI know we are done with this filter */
    notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------


QVector<size_t> DetermineStitchingCoordinatesGeneric::extractTileIndices(QString DataArrayName)
{
    QVector<size_t> tileList(m_PointerList.size());
    DataArrayPath tempPath;
    IDataArray::Pointer iDataArray = IDataArray::NullPointer();
    Int8ArrayType::Pointer MetaDataPtr = Int8ArrayType::NullPointer();
    tempPath.update(getMetaDataAttributeMatrixName().getDataContainerName(), getMetaDataAttributeMatrixName().getAttributeMatrixName(), DataArrayName);
    iDataArray = getDataContainerArray()->getExistingPrereqArrayFromPath<DataArray<int8_t>, AbstractFilter>(this, tempPath);
    MetaDataPtr = boost::dynamic_pointer_cast<DataArray<int8_t> >(iDataArray);
    int8_t dims = MetaDataPtr->getComponentDimensions()[0];
    int8_t* MetaData = MetaDataPtr->getPointer(0);
    QVector<size_t> cDims = MetaDataPtr->getComponentDimensions();
    std::stringstream str;
    for (size_t i=0; i < m_PointerList.size(); i++)
    {

        for (size_t j=0; j<cDims[0]; j++)
        {
            char test = char(MetaData[(cDims[0]*i+j)]);
            str << test;
        }
        //        char test = char(MetaData[(2*i)]);
        //        str << test;
        str >> tileList[i];
        str.str("");
        str.clear();
    }

    return tileList;
}

QVector<float> DetermineStitchingCoordinatesGeneric::extractGlobalIndices(QString DataArrayName, QString Resolution)
{
    QVector<float> globalIndexList(m_PointerList.size());
    QVector<size_t> cDims;
    DataArrayPath tempPath;
    IDataArray::Pointer iDataArray = IDataArray::NullPointer();
    Int8ArrayType::Pointer MetaDataPtr = Int8ArrayType::NullPointer();
    int8_t* MetaData;
    std::stringstream str;

    tempPath.update(getMetaDataAttributeMatrixName().getDataContainerName(), getMetaDataAttributeMatrixName().getAttributeMatrixName(), Resolution);
    iDataArray = getDataContainerArray()->getExistingPrereqArrayFromPath<DataArray<int8_t>, AbstractFilter>(this, tempPath);
    MetaDataPtr = boost::dynamic_pointer_cast<DataArray<int8_t> >(iDataArray);
    MetaData = MetaDataPtr->getPointer(0);
    cDims = MetaDataPtr->getComponentDimensions();
    float resolution;

    for (size_t j=0; j<cDims[0]; j++)
    {
        char test = char(MetaData[(j)]);
        str << test;
    }
    str >> resolution;
    str.str("");
    str.clear();



    tempPath.update(getMetaDataAttributeMatrixName().getDataContainerName(), getMetaDataAttributeMatrixName().getAttributeMatrixName(), DataArrayName);
    iDataArray = getDataContainerArray()->getExistingPrereqArrayFromPath<DataArray<int8_t>, AbstractFilter>(this, tempPath);
    MetaDataPtr = boost::dynamic_pointer_cast<DataArray<int8_t> >(iDataArray);
    MetaData = MetaDataPtr->getPointer(0);

    cDims = MetaDataPtr->getComponentDimensions();



    for (size_t i=0; i < m_PointerList.size(); i++)
    {
        for (size_t j=0; j<cDims[0]; j++)
        {
            char test = char(MetaData[(cDims[0]*i+j)]);
            str << test;
        }
        str >> globalIndexList[i];
        str.str("");
        str.clear();
        globalIndexList[i] = globalIndexList[i]/resolution;
    }





    return globalIndexList;
}







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

