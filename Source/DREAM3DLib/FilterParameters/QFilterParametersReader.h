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

#ifndef _QFilterParametersReader_H_
#define _QFilterParametersReader_H_



#include <QtCore/QString>
#include <QtCore/QSettings>

#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/Common/DREAM3DSetGetMacros.h"
#include "DREAM3DLib/Common/AbstractFilter.h"
#include "DREAM3DLib/Common/FilterPipeline.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/FilterParameter.h"
#include "DREAM3DLib/DataContainers/DataContainerArrayProxy.h"

/*
 *
 */
class DREAM3DLib_EXPORT QFilterParametersReader : public AbstractFilterParametersReader
{
  public:
    DREAM3D_SHARED_POINTERS(QFilterParametersReader)
    DREAM3D_STATIC_NEW_MACRO(QFilterParametersReader)
    DREAM3D_TYPE_MACRO_SUPER(QFilterParametersReader, AbstractFilterParametersReader)


    virtual ~QFilterParametersReader();

    /**
     * @brief ReadPipelineFromFile Reads the QSettings formatted file and returns a FilterPipeline object
     * that contains all the filters that could be found. If a filter can not be found then that filter is simply skipped.
     * If the IObserver is NON-Null then an error message will be passed to it with an error message
     * @param filePath The path to the INI formatted file
     * @param format The format of the file which is anything that QSettings understands
     * @param obs An IObserver object to report errors.
     * @return Shared Pointer to a FilterPipeline Instance
     */
    static FilterPipeline::Pointer ReadPipelineFromFile(QString filePath, QSettings::Format format, IObserver* obs = NULL);


    QSettings* getPrefs();

    void openFile(QString filename, QSettings::Format format);
    void closeFile();

    virtual int openFilterGroup(AbstractFilter* unused, int index);
    virtual int closeFilterGroup();

    virtual QString readString(const QString name, QString value);
    virtual QVector<QString> readStrings(const QString name, QVector<QString> value);

    virtual int8_t readValue(const QString name, int8_t value);
    virtual int16_t readValue(const QString name, int16_t value);
    virtual int32_t readValue(const QString name, int32_t value);
    virtual int64_t readValue(const QString name, int64_t value);
    virtual uint8_t readValue(const QString name, uint8_t value);
    virtual uint16_t readValue(const QString name, uint16_t value);
    virtual uint32_t readValue(const QString name, uint32_t value);
    virtual uint64_t readValue(const QString name, uint64_t value);
    virtual float readValue(const QString name, float def);
    virtual double readValue(const QString name, double value);
    virtual bool readValue(const QString name, bool value);

    virtual QVector<int8_t> readArray(const QString name, QVector<int8_t> value);
    virtual QVector<int16_t> readArray(const QString name, QVector<int16_t> value);
    virtual QVector<int32_t> readArray(const QString name, QVector<int32_t> value);
    virtual QVector<int64_t> readArray(const QString name, QVector<int64_t> value);
    virtual QVector<uint8_t> readArray(const QString name, QVector<uint8_t> value);
    virtual QVector<uint16_t> readArray(const QString name, QVector<uint16_t> value);
    virtual QVector<uint32_t> readArray(const QString name, QVector<uint32_t> value);
    virtual QVector<uint64_t> readArray(const QString name, QVector<uint64_t> value);
    virtual QVector<float> readArray(const QString name, QVector<float> value);
    virtual QVector<double> readArray(const QString name, QVector<double> value);


    virtual IntVec3_t readIntVec3(const QString name, IntVec3_t v);
    virtual FloatVec3_t readFloatVec3(const QString name, FloatVec3_t v);
    virtual FloatVec4_t readFloatVec4(const QString name, FloatVec4_t v);
    virtual FloatVec21_t readFloatVec21(const QString name, FloatVec21_t v);
    virtual Float2ndOrderPoly_t readFloat2ndOrderPoly(const QString name, Float2ndOrderPoly_t v);
    virtual Float3rdOrderPoly_t readFloat3rdOrderPoly(const QString name, Float3rdOrderPoly_t v);
    virtual Float4thOrderPoly_t readFloat4thOrderPoly(const QString name, Float4thOrderPoly_t v);
    virtual FileListInfo_t readFileListInfo(const QString name, FileListInfo_t v);
    virtual ComparisonInput_t readComparisonInput(const QString name, ComparisonInput_t v, int vectorPos);
    virtual ComparisonInputs readComparisonInputs(const QString name, ComparisonInputs v);
    virtual AxisAngleInput_t readAxisAngle(const QString name, AxisAngleInput_t v, int vectorPos);
    virtual QVector<AxisAngleInput_t> readAxisAngles(const QString name, QVector<AxisAngleInput_t> v);
    virtual QSet<QString> readArraySelections(const QString name, QSet<QString> v);

    virtual DataContainerArrayProxy readDataContainerArrayProxy(const QString& name, DataContainerArrayProxy v);
    virtual DataArrayPath readDataArrayPath(const QString& name, DataArrayPath v);
    virtual QVector<DataArrayPath> readDataArrayPathVector(const QString& name, QVector<DataArrayPath> def);
    virtual DynamicTableData readDynamicTableData(const QString& name, DynamicTableData def);

  protected:
    QFilterParametersReader();

  private:
    QSettings* m_Prefs;

    QFilterParametersReader(const QFilterParametersReader&); // Copy Constructor Not Implemented
    void operator=(const QFilterParametersReader&); // Operator '=' Not Implemented
};

#endif /* _QFilterParametersReader_H_ */

