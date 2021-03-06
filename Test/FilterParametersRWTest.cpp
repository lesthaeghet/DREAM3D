/* ============================================================================
* Copyright (c) 2010, Michael A. Jackson (BlueQuartz Software)
* Copyright (c) 2010, Dr. Michael A. Groeber (US Air Force Research Laboratories
* All rights reserved.
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
* Neither the name of Michael A. Groeber, Michael A. Jackson, the US Air Force,
* BlueQuartz Software nor the names of its contributors may be used to endorse
* or promote products derived from this software without specific prior written
* permission.
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
*  This code was written under United States Air Force Contract number
*                           FA8650-07-D-5800
*
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QSet>

#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/Common/DREAM3DSetGetMacros.h"
#include "DREAM3DLib/DataArrays/DataArray.hpp"
#include "DREAM3DLib/Common/FilterPipeline.h"
#include "DREAM3DLib/Common/FilterManager.h"
#include "DREAM3DLib/Common/FilterFactory.hpp"
#include "DREAM3DLib/Plugin/IDREAM3DPlugin.h"
#include "DREAM3DLib/Plugin/DREAM3DPluginLoader.h"
#include "DREAM3DLib/Utilities/UnitTestSupport.hpp"
#include "DREAM3DLib/Utilities/QMetaObjectUtilities.h"
#include "DREAM3DLib/FilterParameters/JsonFilterParametersWriter.h"
#include "DREAM3DLib/FilterParameters/JsonFilterParametersReader.h"
#include "DREAM3DLib/DataContainers/DataArrayPath.h"
#include "DREAM3DLib/DataContainers/DataContainerArrayProxy.h"
#include "DREAM3DLib/FilterParameters/DynamicTableData.h"

#include "DREAM3DTestFileLocations.h"

namespace StringTest
{
  QString Key = "Test String";

  QString Value = "I am testing the string";
};

namespace StringVectorTest
{
  QString Key = "Test Vector";

  QString Value1 = "VecString1";
  QString Value2 = "VecString2";
  QString Value3 = "VecString3";
  QString Value4 = "VecString4";
};

namespace Int8Test
{
  QString Key = "int8_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_8";
  QString ErrorValueMinKey = "Error_Value_Min_8";

  int8_t Value = 1;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<int8_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<int8_t>().min()) - 1;
};

namespace Int16Test
{
  QString Key = "int16_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_16";
  QString ErrorValueMinKey = "Error_Value_Min_16";

  int16_t Value = 2;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<int16_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<int16_t>().min()) - 1;
};

namespace Int32Test
{
  QString Key = "int32_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_32";
  QString ErrorValueMinKey = "Error_Value_Min_32";

  int32_t Value = 3;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<int32_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<int32_t>().min()) - 1;
};

namespace Int64Test
{
  QString Key = "int64_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_64";
  QString ErrorValueMinKey = "Error_Value_Min_64";

  int64_t Value = 4;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<int64_t>().max()) + 1e4;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<int64_t>().min()) - 1e4;
};

namespace uInt8Test
{
  QString Key = "uint8_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_u8";
  QString ErrorValueMinKey = "Error_Value_Min_u8";

  uint8_t Value = 5;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<uint8_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<uint8_t>().min()) - 1;
};

namespace uInt16Test
{
  QString Key = "uint16_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_u16";
  QString ErrorValueMinKey = "Error_Value_Min_u16";

  uint16_t Value = 6;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<uint16_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<uint16_t>().min()) - 1;
};

namespace uInt32Test
{
  QString Key = "uint32_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_u32";
  QString ErrorValueMinKey = "Error_Value_Min_u32";

  uint32_t Value = 7;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<uint32_t>().max()) + 1;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<uint32_t>().min()) - 1;
};

namespace uInt64Test
{
  QString Key = "uint64_t test";
  QString ErrorValueMaxKey = "Error_Value_Max_u64";
  QString ErrorValueMinKey = "Error_Value_Min_u64";

  uint64_t Value = 8;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<uint64_t>().max()) + 1e4;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<uint64_t>().min()) - 1e4;
};

namespace DoubleTest
{
  QString Key = "double test";
  QString ErrorValueMaxKey = "Error_Value_Max_double";
  QString ErrorValueMinKey = "Error_Value_Min_double";

  double Value = 9.91235;
};

namespace FloatTest
{
  QString Key = "float test";
  QString ErrorValueMaxKey = "Error_Value_Max_float";
  QString ErrorValueMinKey = "Error_Value_Min_float";

  float Value = 10.287f;
  double ErrorValueMax = static_cast<double>(std::numeric_limits<float>().max()) + 1e23;
  double ErrorValueMin = static_cast<double>(std::numeric_limits<float>().min()) - 1e23;
};

namespace Int8VectorTest
{
  QString Key = "QVector<int8_t> test";

  const int8_t Value1 = 11;
  const int8_t Value2 = 12;
  const int8_t Value3 = 13;
  const int8_t Value4 = 14;
  const int8_t Value5 = 15;
};

namespace Int16VectorTest
{
  QString Key = "QVector<int16_t> test";

  const int16_t Value1 = 11;
  const int16_t Value2 = 12;
  const int16_t Value3 = 13;
  const int16_t Value4 = 14;
  const int16_t Value5 = 15;
};

namespace Int32VectorTest
{
  QString Key = "QVector<int32_t> test";

  const int32_t Value1 = 11;
  const int32_t Value2 = 12;
  const int32_t Value3 = 13;
  const int32_t Value4 = 14;
  const int32_t Value5 = 15;
};

namespace Int64VectorTest
{
  QString Key = "QVector<int64_t> test";

  const int64_t Value1 = 11;
  const int64_t Value2 = 12;
  const int64_t Value3 = 13;
  const int64_t Value4 = 14;
  const int64_t Value5 = 15;
};

namespace uInt8VectorTest
{
  QString Key = "QVector<uint8_t> test";

  const uint8_t Value1 = 11;
  const uint8_t Value2 = 12;
  const uint8_t Value3 = 13;
  const uint8_t Value4 = 14;
  const uint8_t Value5 = 15;
};

namespace uInt16VectorTest
{
  QString Key = "QVector<uint16_t> test";

  const uint16_t Value1 = 11;
  const uint16_t Value2 = 12;
  const uint16_t Value3 = 13;
  const uint16_t Value4 = 14;
  const uint16_t Value5 = 15;
};

namespace uInt32VectorTest
{
  QString Key = "QVector<uint32_t> test";

  const uint32_t Value1 = 11;
  const uint32_t Value2 = 12;
  const uint32_t Value3 = 13;
  const uint32_t Value4 = 14;
  const uint32_t Value5 = 15;
};

namespace uInt64VectorTest
{
  QString Key = "QVector<uint64_t> test";

  const uint64_t Value1 = 11;
  const uint64_t Value2 = 12;
  const uint64_t Value3 = 13;
  const uint64_t Value4 = 14;
  const uint64_t Value5 = 15;
};

namespace DoubleVectorTest
{
  QString Key = "QVector<double> test";

  const double Value1 = 11;
  const double Value2 = 12;
  const double Value3 = 13;
  const double Value4 = 14;
  const double Value5 = 15;
};

namespace FloatVectorTest
{
  QString Key = "QVector<float> test";

  const float Value1 = 11.2f;
  const float Value2 = 12.3f;
  const float Value3 = 13.4f;
  const float Value4 = 14.5f;
  const float Value5 = 15.6f;
};

namespace IntVec3Test
{
  QString Key = "IntVec3_t test";

  int X = 1;
  int Y = 2;
  int Z = 3;
};

namespace FloatVec3Test
{
  QString Key = "FloatVec3_t test";

  float X = 1.2f;
  float Y = 2.65f;
  float Z = 3.975f;
};

namespace FloatVec4Test
{
  QString Key = "FloatVec4_t test";

  float A = 1.2f;
  float B = 2.65f;
  float C = 3.975f;
  float D = 4.6876f;
};

namespace FloatVec21Test
{
  QString Key = "FloatVec21_t test";

  float Val1 = 1.2f;
  float Val2 = 2.65f;
  float Val3 = 3.975f;
  float Val4 = 4.6876f;
  float Val5 = 1.2f;
  float Val6 = 2.65f;
  float Val7 = 3.975f;
  float Val8 = 4.6876f;
  float Val9 = 1.2f;
  float Val10 = 2.65f;
  float Val11 = 3.975f;
  float Val12 = 4.6876f;
  float Val13 = 1.2f;
  float Val14 = 2.65f;
  float Val15 = 3.975f;
  float Val16 = 4.6876f;
  float Val17 = 1.2f;
  float Val18 = 2.65f;
  float Val19 = 3.975f;
  float Val20 = 4.6876f;
  float Val21 = 1.2f;
};

namespace Float2ndOrderPolyTest
{
  QString Key = "Float2ndOrderPoly_t test";

  float Val1 = 1.2f;
  float Val2 = 2.65f;
  float Val3 = 3.975f;
  float Val4 = 4.6876f;
  float Val5 = 1.2f;
  float Val6 = 2.65f;
};

namespace Float3rdOrderPolyTest
{
  QString Key = "Float3rdOrderPoly_t test";

  float Val1 = 1.2f;
  float Val2 = 2.65f;
  float Val3 = 3.975f;
  float Val4 = 4.6876f;
  float Val5 = 1.2f;
  float Val6 = 1.2f;
  float Val7 = 2.65f;
  float Val8 = 3.975f;
  float Val9 = 4.6876f;
  float Val10 = 1.2f;
};

namespace Float4thOrderPolyTest
{
  QString Key = "Float4thOrderPoly_t test";

  float Val1 = 1.2f;
  float Val2 = 2.65f;
  float Val3 = 3.975f;
  float Val4 = 4.6876f;
  float Val5 = 1.2f;
  float Val6 = 1.2f;
  float Val7 = 2.65f;
  float Val8 = 3.975f;
  float Val9 = 4.6876f;
  float Val10 = 1.2f;
  float Val11 = 1.2f;
  float Val12 = 2.65f;
  float Val13 = 3.975f;
  float Val14 = 4.6876f;
  float Val15 = 1.2f;
};

namespace FileListInfoTest
{
  QString Key = "FileListInfo_t test";

  qint32 PaddingDigits = -1;
  quint32 Ordering = 2;
  qint32 StartIndex = 66;
  qint32 EndIndex = 2345654;
  QString InputPath = "TestString1";
  QString FilePrefix = "TestString2";
  QString FileSuffix = "TestString3";
  QString FileExtension = "TestString4";
};

namespace ComparisonInputTest
{
  QString Key = "ComparisonInput_t test";

  QString DataContainerName = "DataContainer1";
  QString AttributeMatrixName = "AttributeMatrix1";
  QString AttributeArrayName = "AttributeArray1";
  int CompOperator = 3;
  double CompValue = 4.6876;
};

namespace MultipleComparisonInputTest
{
  QString Key = "ComparisonInputs test";

  QString DataContainerName1 = "DataContainer1";
  QString AttributeMatrixName1 = "AttributeMatrix1";
  QString AttributeArrayName1 = "AttributeArray1";
  int CompOperator1 = 3;
  double CompValue1 = 4.6876;

  QString DataContainerName2 = "DataContainer2";
  QString AttributeMatrixName2 = "AttributeMatrix2";
  QString AttributeArrayName2 = "AttributeArray2";
  int CompOperator2 = 4;
  double CompValue2 = 2.935;
};

namespace AxisAngleInputTest
{
  QString Key = "AxisAngleInput_t test";

  float Angle = 0.234f;
  float H = 1.23f;
  float K = 4.32f;
  float L = 3.54f;
};

namespace MultipleAxisAngleInputTest
{
  QString Key = "QVector<AxisAngleInput_t> test";

  float Angle1 = 0.234f;
  float H1 = 1.23f;
  float K1 = 4.32f;
  float L1 = 3.54f;

  float Angle2 = 6.32f;
  float H2 = 9.5677f;
  float K2 = 4.8124f;
  float L2 = 6.315f;
};

namespace StringSetTest
{
  QString Key = "QSet<QString> test";

  QString String1 = "StringSetTest-String1";
  QString String2 = "StringSetTest-String2";
  QString String3 = "StringSetTest-String3";
  QString String4 = "StringSetTest-String4";
};

namespace DataContainerArrayProxyTest
{
  QString Key = "DataContainerArrayProxy test";

  QString DcProxyName = "DataContainer1";
  QString AmProxyName1 = "AttributeMatrix1";
  QString AmProxyName2 = "AttributeMatrix2";
  QString DaProxyName1 = "AttributeArray1";
  QString DaProxyName2 = "AttributeArray2";
  QString DaProxyName3 = "AttributeArray3";
};

namespace DataArrayPathTest
{
  QString Key = "DataArrayPath test";

  QString DCName = "DataContainer";
  QString AMName = "AttributeMatrix";
  QString DAName = "AttributeArray";
};

namespace MultipleDataArrayPathTest
{
  QString Key = "QVector<DataArrayPath> test";

  QString DCName1 = "DataContainer1";
  QString AMName1 = "AttributeMatrix1";
  QString DAName1 = "AttributeArray1";

  QString DCName2 = "DataContainer2";
  QString AMName2 = "AttributeMatrix2";
  QString DAName2 = "AttributeArray2";
};

namespace DynamicTableDataTest
{
  QString Key = "DynamicTableData test";

  int NumRows = 6;
  int NumColumns = 8;
  QString RowHeader1 = "Row 1";
  QString RowHeader2 = "Row 2";
  QString RowHeader3 = "Row 3";
  QString RowHeader4 = "Row 4";
  QString RowHeader5 = "Row 5";
  QString RowHeader6 = "Row 6";
  QString ColumnHeader1 = "Column 1";
  QString ColumnHeader2 = "Column 2";
  QString ColumnHeader3 = "Column 3";
  QString ColumnHeader4 = "Column 4";
  QString ColumnHeader5 = "Column 5";
  QString ColumnHeader6 = "Column 6";
  QString ColumnHeader7 = "Column 7";
  QString ColumnHeader8 = "Column 8";
};

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void RemoveTestFiles()
{
#if REMOVE_TEST_FILES
  QFile::remove(UnitTest::FilterParametersRWTest::OutputFile);
#endif
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TestJsonWriter()
{
  JsonFilterParametersWriter::Pointer writer = JsonFilterParametersWriter::New();
  writer->setFileName(UnitTest::FilterParametersRWTest::OutputFile);
  writer->setPipelineName("TestPipelineName");

  int err = 0;

  err = writer->openFilterGroup(NULL, 0);
  DREAM3D_REQUIRE_EQUAL(err, 0)

  // QString write test
  {
    err = writer->writeValue(StringTest::Key, StringTest::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<QString> write test
  {
    QVector<QString> vector;
    vector.push_back(StringVectorTest::Value1);
    vector.push_back(StringVectorTest::Value2);
    vector.push_back(StringVectorTest::Value3);
    vector.push_back(StringVectorTest::Value4);
    err = writer->writeValue(StringVectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // int8_t write test
  {
    err = writer->writeValue(Int8Test::Key, Int8Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int8Test::ErrorValueMaxKey, Int8Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int8Test::ErrorValueMinKey, Int8Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // int16_t write test
  {
    err = writer->writeValue(Int16Test::Key, Int16Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int16Test::ErrorValueMaxKey, Int16Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int16Test::ErrorValueMinKey, Int16Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // int32_t write test
  {
    err = writer->writeValue(Int32Test::Key, Int32Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int32Test::ErrorValueMaxKey, Int32Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int32Test::ErrorValueMinKey, Int32Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // int64_t write test
  {
    err = writer->writeValue(Int64Test::Key, Int64Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int64Test::ErrorValueMaxKey, Int64Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(Int64Test::ErrorValueMinKey, Int64Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // uint8_t write test
  {
    err = writer->writeValue(uInt8Test::Key, uInt8Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt8Test::ErrorValueMaxKey, uInt8Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt8Test::ErrorValueMinKey, uInt8Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // uint16_t write test
  {
    err = writer->writeValue(uInt16Test::Key, uInt16Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt16Test::ErrorValueMaxKey, uInt16Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt16Test::ErrorValueMinKey, uInt16Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // uint32_t write test
  {
    err = writer->writeValue(uInt32Test::Key, uInt32Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt32Test::ErrorValueMaxKey, uInt32Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt32Test::ErrorValueMinKey, uInt32Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // uint64_t write test
  {
    err = writer->writeValue(uInt64Test::Key, uInt64Test::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt64Test::ErrorValueMaxKey, uInt64Test::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(uInt64Test::ErrorValueMinKey, uInt64Test::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // double write test
  {
    err = writer->writeValue(DoubleTest::Key, DoubleTest::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // float write test
  {
    err = writer->writeValue(FloatTest::Key, FloatTest::Value);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(FloatTest::ErrorValueMaxKey, FloatTest::ErrorValueMax);
    DREAM3D_REQUIRE_EQUAL(err, 0)

      err = writer->writeValue(FloatTest::ErrorValueMinKey, FloatTest::ErrorValueMin);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<int8_t> write test
  {
    QVector<int8_t> vector;
    vector.push_back(Int8VectorTest::Value1);
    vector.push_back(Int8VectorTest::Value2);
    vector.push_back(Int8VectorTest::Value3);
    vector.push_back(Int8VectorTest::Value4);
    vector.push_back(Int8VectorTest::Value5);
    err = writer->writeValue(Int8VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<int16_t> write test
  {
    QVector<int16_t> vector;
    vector.push_back(Int16VectorTest::Value1);
    vector.push_back(Int16VectorTest::Value2);
    vector.push_back(Int16VectorTest::Value3);
    vector.push_back(Int16VectorTest::Value4);
    vector.push_back(Int16VectorTest::Value5);
    err = writer->writeValue(Int16VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<int32_t> write test
  {
    QVector<int32_t> vector;
    vector.push_back(Int32VectorTest::Value1);
    vector.push_back(Int32VectorTest::Value2);
    vector.push_back(Int32VectorTest::Value3);
    vector.push_back(Int32VectorTest::Value4);
    vector.push_back(Int32VectorTest::Value5);
    err = writer->writeValue(Int32VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<int64_t> write test
  {
    QVector<int64_t> vector;
    vector.push_back(Int64VectorTest::Value1);
    vector.push_back(Int64VectorTest::Value2);
    vector.push_back(Int64VectorTest::Value3);
    vector.push_back(Int64VectorTest::Value4);
    vector.push_back(Int64VectorTest::Value5);
    err = writer->writeValue(Int64VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<uint8_t> write test
  {
    QVector<uint8_t> vector;
    vector.push_back(uInt8VectorTest::Value1);
    vector.push_back(uInt8VectorTest::Value2);
    vector.push_back(uInt8VectorTest::Value3);
    vector.push_back(uInt8VectorTest::Value4);
    vector.push_back(uInt8VectorTest::Value5);
    err = writer->writeValue(uInt8VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<uint16_t> write test
  {
    QVector<uint16_t> vector;
    vector.push_back(uInt16VectorTest::Value1);
    vector.push_back(uInt16VectorTest::Value2);
    vector.push_back(uInt16VectorTest::Value3);
    vector.push_back(uInt16VectorTest::Value4);
    vector.push_back(uInt16VectorTest::Value5);
    err = writer->writeValue(uInt16VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<uint32_t> write test
  {
    QVector<uint32_t> vector;
    vector.push_back(uInt32VectorTest::Value1);
    vector.push_back(uInt32VectorTest::Value2);
    vector.push_back(uInt32VectorTest::Value3);
    vector.push_back(uInt32VectorTest::Value4);
    vector.push_back(uInt32VectorTest::Value5);
    err = writer->writeValue(uInt32VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<uint64_t> write test
  {
    QVector<uint64_t> vector;
    vector.push_back(uInt64VectorTest::Value1);
    vector.push_back(uInt64VectorTest::Value2);
    vector.push_back(uInt64VectorTest::Value3);
    vector.push_back(uInt64VectorTest::Value4);
    vector.push_back(uInt64VectorTest::Value5);
    err = writer->writeValue(uInt64VectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<double> write test
  {
    QVector<double> vector;
    vector.push_back(DoubleVectorTest::Value1);
    vector.push_back(DoubleVectorTest::Value2);
    vector.push_back(DoubleVectorTest::Value3);
    vector.push_back(DoubleVectorTest::Value4);
    vector.push_back(DoubleVectorTest::Value5);
    err = writer->writeValue(DoubleVectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<float> write test
  {
    QVector<float> vector;
    vector.push_back(FloatVectorTest::Value1);
    vector.push_back(FloatVectorTest::Value2);
    vector.push_back(FloatVectorTest::Value3);
    vector.push_back(FloatVectorTest::Value4);
    vector.push_back(FloatVectorTest::Value5);
    err = writer->writeValue(FloatVectorTest::Key, vector);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // IntVec3_t write test
  {
    IntVec3_t vec3;
    vec3.x = IntVec3Test::X;
    vec3.y = IntVec3Test::Y;
    vec3.z = IntVec3Test::Z;
    err = writer->writeValue(IntVec3Test::Key, vec3);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // FloatVec3_t write test
  {
    FloatVec3_t vec3;
    vec3.x = FloatVec3Test::X;
    vec3.y = FloatVec3Test::Y;
    vec3.z = FloatVec3Test::Z;
    err = writer->writeValue(FloatVec3Test::Key, vec3);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // FloatVec4_t write test
  {
    FloatVec4_t vec4;
    vec4.a = FloatVec4Test::A;
    vec4.b = FloatVec4Test::B;
    vec4.c = FloatVec4Test::C;
    vec4.d = FloatVec4Test::D;
    err = writer->writeValue(FloatVec4Test::Key, vec4);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // FloatVec21_t write test
  {
    FloatVec21_t vec21;
    vec21.v11 = FloatVec21Test::Val1;
    vec21.v12 = FloatVec21Test::Val2;
    vec21.v13 = FloatVec21Test::Val3;
    vec21.v14 = FloatVec21Test::Val4;
    vec21.v15 = FloatVec21Test::Val5;
    vec21.v16 = FloatVec21Test::Val6;
    vec21.v22 = FloatVec21Test::Val7;
    vec21.v23 = FloatVec21Test::Val8;
    vec21.v24 = FloatVec21Test::Val9;
    vec21.v25 = FloatVec21Test::Val10;
    vec21.v26 = FloatVec21Test::Val11;
    vec21.v33 = FloatVec21Test::Val12;
    vec21.v34 = FloatVec21Test::Val13;
    vec21.v35 = FloatVec21Test::Val14;
    vec21.v36 = FloatVec21Test::Val15;
    vec21.v44 = FloatVec21Test::Val16;
    vec21.v45 = FloatVec21Test::Val17;
    vec21.v46 = FloatVec21Test::Val18;
    vec21.v55 = FloatVec21Test::Val19;
    vec21.v56 = FloatVec21Test::Val20;
    vec21.v66 = FloatVec21Test::Val21;

    err = writer->writeValue(FloatVec21Test::Key, vec21);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // Float2ndOrderPoly_t write test
  {
    Float2ndOrderPoly_t poly;
    poly.c20 = Float2ndOrderPolyTest::Val1;
    poly.c02 = Float2ndOrderPolyTest::Val2;
    poly.c11 = Float2ndOrderPolyTest::Val3;
    poly.c10 = Float2ndOrderPolyTest::Val4;
    poly.c01 = Float2ndOrderPolyTest::Val5;
    poly.c00 = Float2ndOrderPolyTest::Val6;

    err = writer->writeValue(Float2ndOrderPolyTest::Key, poly);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // Float3rdOrderPoly_t write test
  {
    Float3rdOrderPoly_t poly;
    poly.c30 = Float3rdOrderPolyTest::Val1;
    poly.c03 = Float3rdOrderPolyTest::Val2;
    poly.c21 = Float3rdOrderPolyTest::Val3;
    poly.c12 = Float3rdOrderPolyTest::Val4;
    poly.c20 = Float3rdOrderPolyTest::Val5;
    poly.c02 = Float3rdOrderPolyTest::Val6;
    poly.c11 = Float3rdOrderPolyTest::Val7;
    poly.c10 = Float3rdOrderPolyTest::Val8;
    poly.c01 = Float3rdOrderPolyTest::Val9;
    poly.c00 = Float3rdOrderPolyTest::Val10;

    err = writer->writeValue(Float3rdOrderPolyTest::Key, poly);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // Float4thOrderPoly_t write test
  {
    Float4thOrderPoly_t poly;
    poly.c02 = Float4thOrderPolyTest::Val1;
    poly.c11 = Float4thOrderPolyTest::Val2;
    poly.c10 = Float4thOrderPolyTest::Val3;
    poly.c01 = Float4thOrderPolyTest::Val4;
    poly.c00 = Float4thOrderPolyTest::Val5;
    poly.c40 = Float4thOrderPolyTest::Val6;
    poly.c04 = Float4thOrderPolyTest::Val7;
    poly.c31 = Float4thOrderPolyTest::Val8;
    poly.c13 = Float4thOrderPolyTest::Val9;
    poly.c22 = Float4thOrderPolyTest::Val10;
    poly.c30 = Float4thOrderPolyTest::Val11;
    poly.c03 = Float4thOrderPolyTest::Val12;
    poly.c21 = Float4thOrderPolyTest::Val13;
    poly.c12 = Float4thOrderPolyTest::Val14;
    poly.c20 = Float4thOrderPolyTest::Val15;

    err = writer->writeValue(Float4thOrderPolyTest::Key, poly);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // FileListInfo_t write test
  {
    FileListInfo_t info;
    info.PaddingDigits = FileListInfoTest::PaddingDigits;
    info.Ordering = FileListInfoTest::Ordering;
    info.StartIndex = FileListInfoTest::StartIndex;
    info.EndIndex = FileListInfoTest::EndIndex;
    info.InputPath = FileListInfoTest::InputPath;
    info.FilePrefix = FileListInfoTest::FilePrefix;
    info.FileSuffix = FileListInfoTest::FileSuffix;
    info.FileExtension = FileListInfoTest::FileExtension;

    err = writer->writeValue(FileListInfoTest::Key, info);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // ComparisonInput_t write test
  {
    ComparisonInput_t input;
    input.dataContainerName = ComparisonInputTest::DataContainerName;
    input.attributeMatrixName = ComparisonInputTest::AttributeMatrixName;
    input.attributeArrayName = ComparisonInputTest::AttributeArrayName;
    input.compOperator = ComparisonInputTest::CompOperator;
    input.compValue = ComparisonInputTest::CompValue;

    err = writer->writeValue(ComparisonInputTest::Key, input);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // ComparisonInputs write test
  {
    ComparisonInputs inputs;

    ComparisonInput_t input1;
    input1.dataContainerName = MultipleComparisonInputTest::DataContainerName1;
    input1.attributeMatrixName = MultipleComparisonInputTest::AttributeMatrixName1;
    input1.attributeArrayName = MultipleComparisonInputTest::AttributeArrayName1;
    input1.compOperator = MultipleComparisonInputTest::CompOperator1;
    input1.compValue = MultipleComparisonInputTest::CompValue1;
    inputs.addInput(input1);

    ComparisonInput_t input2;
    input2.dataContainerName = MultipleComparisonInputTest::DataContainerName2;
    input2.attributeMatrixName = MultipleComparisonInputTest::AttributeMatrixName2;
    input2.attributeArrayName = MultipleComparisonInputTest::AttributeArrayName2;
    input2.compOperator = MultipleComparisonInputTest::CompOperator2;
    input2.compValue = MultipleComparisonInputTest::CompValue2;
    inputs.addInput(input2);

    err = writer->writeValue(MultipleComparisonInputTest::Key, inputs);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<AxisAngleInput_t> write test
  {
    QVector<AxisAngleInput_t> inputs;

    AxisAngleInput_t input1;
    input1.angle = MultipleAxisAngleInputTest::Angle1;
    input1.h = MultipleAxisAngleInputTest::H1;
    input1.k = MultipleAxisAngleInputTest::K1;
    input1.l = MultipleAxisAngleInputTest::L1;
    inputs.push_back(input1);

    AxisAngleInput_t input2;
    input2.angle = MultipleAxisAngleInputTest::Angle2;
    input2.h = MultipleAxisAngleInputTest::H2;
    input2.k = MultipleAxisAngleInputTest::K2;
    input2.l = MultipleAxisAngleInputTest::L2;
    inputs.push_back(input2);

    err = writer->writeValue(MultipleAxisAngleInputTest::Key, inputs);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // AxisAngleInput_t write test
  {
    AxisAngleInput_t input;
    input.angle = AxisAngleInputTest::Angle;
    input.h = AxisAngleInputTest::H;
    input.k = AxisAngleInputTest::K;
    input.l = AxisAngleInputTest::L;

    err = writer->writeValue(AxisAngleInputTest::Key, input);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QSet<QString> write test
  {
    QSet<QString> set;
    set << StringSetTest::String1 << StringSetTest::String2 << StringSetTest::String3 << StringSetTest::String4;

    err = writer->writeArraySelections(StringSetTest::Key, set);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // DataContainerArrayProxy write test
  {
    DataContainerArrayProxy proxy;
    DataContainerProxy dcProxy(DataContainerArrayProxyTest::DcProxyName);
    AttributeMatrixProxy amProxy1(DataContainerArrayProxyTest::AmProxyName1);
    AttributeMatrixProxy amProxy2(DataContainerArrayProxyTest::AmProxyName2);
    DataArrayProxy daProxy1(DataContainerArrayProxyTest::DaProxyName1, DataContainerArrayProxyTest::DaProxyName1);
    DataArrayProxy daProxy2(DataContainerArrayProxyTest::DaProxyName2, DataContainerArrayProxyTest::DaProxyName2);
    DataArrayProxy daProxy3(DataContainerArrayProxyTest::DaProxyName3, DataContainerArrayProxyTest::DaProxyName3);
    amProxy1.dataArrays.insert(daProxy1.name, daProxy1);
    amProxy1.dataArrays.insert(daProxy2.name, daProxy2);
    amProxy2.dataArrays.insert(daProxy3.name, daProxy3);
    dcProxy.attributeMatricies.insert(amProxy1.name, amProxy1);
    dcProxy.attributeMatricies.insert(amProxy2.name, amProxy2);
    proxy.dataContainers.insert(dcProxy.name, dcProxy);

    err = writer->writeValue(DataContainerArrayProxyTest::Key, proxy);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // DataArrayPath write test
  {
    DataArrayPath path(DataArrayPathTest::DCName, DataArrayPathTest::AMName, DataArrayPathTest::DAName);

    err = writer->writeValue(DataArrayPathTest::Key, path);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // QVector<DataArrayPath> write test
  {
    QVector<DataArrayPath> paths;
    paths.push_back(DataArrayPath(MultipleDataArrayPathTest::DCName1, MultipleDataArrayPathTest::AMName1, MultipleDataArrayPathTest::DAName1));
    paths.push_back(DataArrayPath(MultipleDataArrayPathTest::DCName2, MultipleDataArrayPathTest::AMName2, MultipleDataArrayPathTest::DAName2));

    err = writer->writeValue(MultipleDataArrayPathTest::Key, paths);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }

  // DynamicTableData write test
  {
    QStringList rHeaders, cHeaders;
    rHeaders << DynamicTableDataTest::RowHeader1 << DynamicTableDataTest::RowHeader2 << DynamicTableDataTest::RowHeader3 << DynamicTableDataTest::RowHeader4;
    rHeaders << DynamicTableDataTest::RowHeader5 << DynamicTableDataTest::RowHeader6;
    cHeaders << DynamicTableDataTest::ColumnHeader1 << DynamicTableDataTest::ColumnHeader2 << DynamicTableDataTest::ColumnHeader3 << DynamicTableDataTest::ColumnHeader4;
    cHeaders << DynamicTableDataTest::ColumnHeader5 << DynamicTableDataTest::ColumnHeader6 << DynamicTableDataTest::ColumnHeader7 << DynamicTableDataTest::ColumnHeader8;
    DynamicTableData data(DynamicTableDataTest::NumRows, DynamicTableDataTest::NumColumns, rHeaders, cHeaders);

    err = writer->writeValue(DynamicTableDataTest::Key, data);
    DREAM3D_REQUIRE_EQUAL(err, 0)
  }


  err = writer->closeFilterGroup();
  DREAM3D_REQUIRE_EQUAL(err, 0)

  return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TestJsonReader()
{
  JsonFilterParametersReader::Pointer reader = JsonFilterParametersReader::New();
  int err = reader->openFile(UnitTest::FilterParametersRWTest::OutputFile);
  DREAM3D_REQUIRE_EQUAL(err, 0)

  err = reader->openFilterGroup(NULL, 0);
  DREAM3D_REQUIRE_EQUAL(err, 0)

  // QString read test
  {
    QString str = reader->readString(StringTest::Key, "");
    DREAM3D_REQUIRE_EQUAL(str, StringTest::Value)
  }

  // QVector<QString> read test
  {
    QVector<QString> vector = reader->readStrings(StringVectorTest::Key, QVector<QString>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 4)
    DREAM3D_REQUIRE_EQUAL(vector[0], StringVectorTest::Value1)
    DREAM3D_REQUIRE_EQUAL(vector[1], StringVectorTest::Value2)
    DREAM3D_REQUIRE_EQUAL(vector[2], StringVectorTest::Value3)
    DREAM3D_REQUIRE_EQUAL(vector[3], StringVectorTest::Value4)
  }

  // int8_t read test
  {
    int8_t def = 0;
    int8_t val = reader->readValue(Int8Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, Int8Test::Value)

    int8_t errorValMax = reader->readValue(Int8Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

    int8_t errorValMin = reader->readValue(Int8Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // int16_t read test
  {
    int16_t def = 0;
    int16_t val = reader->readValue(Int16Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, Int16Test::Value)

      int16_t errorValMax = reader->readValue(Int16Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      int16_t errorValMin = reader->readValue(Int16Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // int32_t read test
  {
    int32_t def = 0;
    int32_t val = reader->readValue(Int32Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, Int32Test::Value)

      int32_t errorValMax = reader->readValue(Int32Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      int32_t errorValMin = reader->readValue(Int32Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // int64_t read test
  {
    int64_t def = 0;
    int64_t val = reader->readValue(Int64Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, Int64Test::Value)

      int64_t errorValMax = reader->readValue(Int64Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      int64_t errorValMin = reader->readValue(Int64Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // uint8_t read test
  {
    uint8_t def = 0;
    uint8_t val = reader->readValue(uInt8Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, uInt8Test::Value)

      uint8_t errorValMax = reader->readValue(uInt8Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      uint8_t errorValMin = reader->readValue(uInt8Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // uint16_t read test
  {
    uint16_t def = 0;
    uint16_t val = reader->readValue(uInt16Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, uInt16Test::Value)

      uint16_t errorValMax = reader->readValue(uInt16Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      uint16_t errorValMin = reader->readValue(uInt16Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // uint32_t read test
  {
    uint32_t def = 0;
    uint32_t val = reader->readValue(uInt32Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, uInt32Test::Value)

      uint32_t errorValMax = reader->readValue(uInt32Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      uint32_t errorValMin = reader->readValue(uInt32Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // uint64_t read test
  {
    uint64_t def = 0;
    uint64_t val = reader->readValue(uInt64Test::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, uInt64Test::Value)

      uint64_t errorValMax = reader->readValue(uInt64Test::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      uint64_t errorValMin = reader->readValue(uInt64Test::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // double read test
  {
    double def = 0;
    double val = reader->readValue(DoubleTest::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, DoubleTest::Value)
  }

  // float read test
  {
    float def = 0;
    float val = reader->readValue(FloatTest::Key, def);
    DREAM3D_REQUIRE_EQUAL(val, FloatTest::Value)

      float errorValMax = reader->readValue(FloatTest::ErrorValueMaxKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMax, def)

      float errorValMin = reader->readValue(FloatTest::ErrorValueMinKey, def);
    DREAM3D_REQUIRE_EQUAL(errorValMin, def)
  }

  // QVector<int8_t> read test
  {
    QVector<int8_t> vector = reader->readArray(Int8VectorTest::Key, QVector<int8_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], Int8VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], Int8VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], Int8VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], Int8VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], Int8VectorTest::Value5)
  }

  // QVector<int16_t> read test
  {
    QVector<int16_t> vector = reader->readArray(Int16VectorTest::Key, QVector<int16_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], Int16VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], Int16VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], Int16VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], Int16VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], Int16VectorTest::Value5)
  }

  // QVector<int32_t> read test
  {
    QVector<int32_t> vector = reader->readArray(Int32VectorTest::Key, QVector<int32_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], Int32VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], Int32VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], Int32VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], Int32VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], Int32VectorTest::Value5)
  }

  // QVector<int64_t> read test
  {
    QVector<int64_t> vector = reader->readArray(Int64VectorTest::Key, QVector<int64_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], Int64VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], Int64VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], Int64VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], Int64VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], Int64VectorTest::Value5)
  }

  // QVector<uint8_t> read test
  {
    QVector<uint8_t> vector = reader->readArray(uInt8VectorTest::Key, QVector<uint8_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], uInt8VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], uInt8VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], uInt8VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], uInt8VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], uInt8VectorTest::Value5)
  }

  // QVector<uint16_t> read test
  {
    QVector<uint16_t> vector = reader->readArray(uInt16VectorTest::Key, QVector<uint16_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], uInt16VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], uInt16VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], uInt16VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], uInt16VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], uInt16VectorTest::Value5)
  }

  // QVector<uint32_t> read test
  {
    QVector<uint32_t> vector = reader->readArray(uInt32VectorTest::Key, QVector<uint32_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], uInt32VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], uInt32VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], uInt32VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], uInt32VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], uInt32VectorTest::Value5)
  }

  // QVector<uint64_t> read test
  {
    QVector<uint64_t> vector = reader->readArray(uInt64VectorTest::Key, QVector<uint64_t>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], uInt64VectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], uInt64VectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], uInt64VectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], uInt64VectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], uInt64VectorTest::Value5)
  }

  // QVector<double> read test
  {
    QVector<double> vector = reader->readArray(DoubleVectorTest::Key, QVector<double>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], DoubleVectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], DoubleVectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], DoubleVectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], DoubleVectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], DoubleVectorTest::Value5)
  }

  // QVector<float> read test
  {
    QVector<float> vector = reader->readArray(FloatVectorTest::Key, QVector<float>());
    DREAM3D_REQUIRE_EQUAL(vector.size(), 5)
      DREAM3D_REQUIRE_EQUAL(vector[0], FloatVectorTest::Value1)
      DREAM3D_REQUIRE_EQUAL(vector[1], FloatVectorTest::Value2)
      DREAM3D_REQUIRE_EQUAL(vector[2], FloatVectorTest::Value3)
      DREAM3D_REQUIRE_EQUAL(vector[3], FloatVectorTest::Value4)
      DREAM3D_REQUIRE_EQUAL(vector[4], FloatVectorTest::Value5)
  }

  // IntVec3_t read test
  {
    IntVec3_t intvec3 = reader->readIntVec3(IntVec3Test::Key, IntVec3_t());
    DREAM3D_REQUIRE_EQUAL(intvec3.x, IntVec3Test::X)
      DREAM3D_REQUIRE_EQUAL(intvec3.y, IntVec3Test::Y)
      DREAM3D_REQUIRE_EQUAL(intvec3.z, IntVec3Test::Z)
  }

  // FloatVec3_t read test
  {
    FloatVec3_t floatvec3 = reader->readFloatVec3(FloatVec3Test::Key, FloatVec3_t());
    DREAM3D_REQUIRE_EQUAL(floatvec3.x, FloatVec3Test::X)
      DREAM3D_REQUIRE_EQUAL(floatvec3.y, FloatVec3Test::Y)
      DREAM3D_REQUIRE_EQUAL(floatvec3.z, FloatVec3Test::Z)
  }

  // FloatVec4_t read test
  {
    FloatVec4_t floatvec4 = reader->readFloatVec4(FloatVec4Test::Key, FloatVec4_t());
    DREAM3D_REQUIRE_EQUAL(floatvec4.a, FloatVec4Test::A)
      DREAM3D_REQUIRE_EQUAL(floatvec4.b, FloatVec4Test::B)
      DREAM3D_REQUIRE_EQUAL(floatvec4.c, FloatVec4Test::C)
      DREAM3D_REQUIRE_EQUAL(floatvec4.d, FloatVec4Test::D)
  }

  // FloatVec21_t read test
  {
    FloatVec21_t floatvec21 = reader->readFloatVec21(FloatVec21Test::Key, FloatVec21_t());
    DREAM3D_REQUIRE_EQUAL(floatvec21.v11, FloatVec21Test::Val1)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v12, FloatVec21Test::Val2)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v13, FloatVec21Test::Val3)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v14, FloatVec21Test::Val4)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v15, FloatVec21Test::Val5)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v16, FloatVec21Test::Val6)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v22, FloatVec21Test::Val7)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v23, FloatVec21Test::Val8)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v24, FloatVec21Test::Val9)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v25, FloatVec21Test::Val10)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v26, FloatVec21Test::Val11)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v33, FloatVec21Test::Val12)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v34, FloatVec21Test::Val13)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v35, FloatVec21Test::Val14)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v36, FloatVec21Test::Val15)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v44, FloatVec21Test::Val16)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v45, FloatVec21Test::Val17)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v46, FloatVec21Test::Val18)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v55, FloatVec21Test::Val19)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v56, FloatVec21Test::Val20)
      DREAM3D_REQUIRE_EQUAL(floatvec21.v66, FloatVec21Test::Val21)
  }

  // Float2ndOrderPoly_t read test
  {
    Float2ndOrderPoly_t floatPoly = reader->readFloat2ndOrderPoly(Float2ndOrderPolyTest::Key, Float2ndOrderPoly_t());
    DREAM3D_REQUIRE_EQUAL(floatPoly.c20, Float2ndOrderPolyTest::Val1)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c02, Float2ndOrderPolyTest::Val2)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c11, Float2ndOrderPolyTest::Val3)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c10, Float2ndOrderPolyTest::Val4)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c01, Float2ndOrderPolyTest::Val5)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c00, Float2ndOrderPolyTest::Val6)
  }

  // Float3rdOrderPoly_t read test
  {
    Float3rdOrderPoly_t floatPoly = reader->readFloat3rdOrderPoly(Float3rdOrderPolyTest::Key, Float3rdOrderPoly_t());
    DREAM3D_REQUIRE_EQUAL(floatPoly.c30, Float3rdOrderPolyTest::Val1)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c03, Float3rdOrderPolyTest::Val2)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c21, Float3rdOrderPolyTest::Val3)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c12, Float3rdOrderPolyTest::Val4)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c20, Float3rdOrderPolyTest::Val5)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c02, Float3rdOrderPolyTest::Val6)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c11, Float3rdOrderPolyTest::Val7)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c10, Float3rdOrderPolyTest::Val8)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c01, Float3rdOrderPolyTest::Val9)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c00, Float3rdOrderPolyTest::Val10)
  }

  // Float4thOrderPoly_t read test
  {
    Float4thOrderPoly_t floatPoly = reader->readFloat4thOrderPoly(Float4thOrderPolyTest::Key, Float4thOrderPoly_t());
    DREAM3D_REQUIRE_EQUAL(floatPoly.c02, Float4thOrderPolyTest::Val1)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c11, Float4thOrderPolyTest::Val2)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c10, Float4thOrderPolyTest::Val3)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c01, Float4thOrderPolyTest::Val4)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c00, Float4thOrderPolyTest::Val5)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c40, Float4thOrderPolyTest::Val6)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c04, Float4thOrderPolyTest::Val7)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c31, Float4thOrderPolyTest::Val8)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c13, Float4thOrderPolyTest::Val9)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c22, Float4thOrderPolyTest::Val10)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c30, Float4thOrderPolyTest::Val11)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c03, Float4thOrderPolyTest::Val12)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c21, Float4thOrderPolyTest::Val13)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c12, Float4thOrderPolyTest::Val14)
      DREAM3D_REQUIRE_EQUAL(floatPoly.c20, Float4thOrderPolyTest::Val15)
  }

  // FileListInfo_t read test
  {
    FileListInfo_t fileListInfo = reader->readFileListInfo(FileListInfoTest::Key, FileListInfo_t());
      DREAM3D_REQUIRE_EQUAL(fileListInfo.EndIndex, FileListInfoTest::EndIndex)
        DREAM3D_REQUIRE_EQUAL(fileListInfo.FileExtension, FileListInfoTest::FileExtension)
        DREAM3D_REQUIRE_EQUAL(fileListInfo.FilePrefix, FileListInfoTest::FilePrefix)
        DREAM3D_REQUIRE_EQUAL(fileListInfo.FileSuffix, FileListInfoTest::FileSuffix)
        DREAM3D_REQUIRE_EQUAL(fileListInfo.InputPath, FileListInfoTest::InputPath)
        DREAM3D_REQUIRE_EQUAL(fileListInfo.Ordering, FileListInfoTest::Ordering)
      DREAM3D_REQUIRE_EQUAL(fileListInfo.PaddingDigits, FileListInfoTest::PaddingDigits)
      DREAM3D_REQUIRE_EQUAL(fileListInfo.StartIndex, FileListInfoTest::StartIndex)
  }

  // ComparisonInputs read test
  {
    ComparisonInputs inputs = reader->readComparisonInputs(MultipleComparisonInputTest::Key, ComparisonInputs());
    DREAM3D_REQUIRE_EQUAL(inputs.size(), 2)

    DREAM3D_REQUIRE_EQUAL(inputs[0].attributeArrayName, MultipleComparisonInputTest::AttributeArrayName1)
      DREAM3D_REQUIRE_EQUAL(inputs[0].attributeMatrixName, MultipleComparisonInputTest::AttributeMatrixName1)
      DREAM3D_REQUIRE_EQUAL(inputs[0].compOperator, MultipleComparisonInputTest::CompOperator1)
      DREAM3D_REQUIRE_EQUAL(inputs[0].compValue, MultipleComparisonInputTest::CompValue1)
      DREAM3D_REQUIRE_EQUAL(inputs[0].dataContainerName, MultipleComparisonInputTest::DataContainerName1)

      DREAM3D_REQUIRE_EQUAL(inputs[1].attributeArrayName, MultipleComparisonInputTest::AttributeArrayName2)
      DREAM3D_REQUIRE_EQUAL(inputs[1].attributeMatrixName, MultipleComparisonInputTest::AttributeMatrixName2)
      DREAM3D_REQUIRE_EQUAL(inputs[1].compOperator, MultipleComparisonInputTest::CompOperator2)
      DREAM3D_REQUIRE_EQUAL(inputs[1].compValue, MultipleComparisonInputTest::CompValue2)
      DREAM3D_REQUIRE_EQUAL(inputs[1].dataContainerName, MultipleComparisonInputTest::DataContainerName2)
  }

  // ComparisonInput_t read test
  {
    ComparisonInput_t input = reader->readComparisonInput(MultipleComparisonInputTest::Key, ComparisonInput_t(), 0);
    DREAM3D_REQUIRE_EQUAL(input.attributeArrayName, MultipleComparisonInputTest::AttributeArrayName1)
    DREAM3D_REQUIRE_EQUAL(input.attributeMatrixName, MultipleComparisonInputTest::AttributeMatrixName1)
    DREAM3D_REQUIRE_EQUAL(input.compOperator, MultipleComparisonInputTest::CompOperator1)
    DREAM3D_REQUIRE_EQUAL(input.compValue, MultipleComparisonInputTest::CompValue1)
    DREAM3D_REQUIRE_EQUAL(input.dataContainerName, MultipleComparisonInputTest::DataContainerName1)
  }

  // QVector<AxisAngleInput_t> read test
  {
    QVector<AxisAngleInput_t> inputs = reader->readAxisAngles(MultipleAxisAngleInputTest::Key, QVector<AxisAngleInput_t>());
    DREAM3D_REQUIRE_EQUAL(inputs.size(), 2)

    DREAM3D_REQUIRE_EQUAL(inputs[0].angle, MultipleAxisAngleInputTest::Angle1)
    DREAM3D_REQUIRE_EQUAL(inputs[0].h, MultipleAxisAngleInputTest::H1)
    DREAM3D_REQUIRE_EQUAL(inputs[0].k, MultipleAxisAngleInputTest::K1)
    DREAM3D_REQUIRE_EQUAL(inputs[0].l, MultipleAxisAngleInputTest::L1)

    DREAM3D_REQUIRE_EQUAL(inputs[1].angle, MultipleAxisAngleInputTest::Angle2)
    DREAM3D_REQUIRE_EQUAL(inputs[1].h, MultipleAxisAngleInputTest::H2)
    DREAM3D_REQUIRE_EQUAL(inputs[1].k, MultipleAxisAngleInputTest::K2)
    DREAM3D_REQUIRE_EQUAL(inputs[1].l, MultipleAxisAngleInputTest::L2)
  }

  // AxisAngleInput_t read test
  {
    AxisAngleInput_t input = reader->readAxisAngle(AxisAngleInputTest::Key, AxisAngleInput_t(), 0);
    DREAM3D_REQUIRE_EQUAL(input.angle, AxisAngleInputTest::Angle)
    DREAM3D_REQUIRE_EQUAL(input.h, AxisAngleInputTest::H)
    DREAM3D_REQUIRE_EQUAL(input.k, AxisAngleInputTest::K)
    DREAM3D_REQUIRE_EQUAL(input.l, AxisAngleInputTest::L)
  }

  // QSet<QString> read test
  {
    QSet<QString> strings = reader->readArraySelections(StringSetTest::Key, QSet<QString>());
    DREAM3D_REQUIRE_EQUAL(strings.size(), 4)

    DREAM3D_REQUIRE_EQUAL(strings.contains(StringSetTest::String1), true)
    DREAM3D_REQUIRE_EQUAL(strings.contains(StringSetTest::String2), true)
    DREAM3D_REQUIRE_EQUAL(strings.contains(StringSetTest::String3), true)
    DREAM3D_REQUIRE_EQUAL(strings.contains(StringSetTest::String4), true)
  }

  // DataContainerArrayProxy read test
  {
    DataContainerArrayProxy proxy = reader->readDataContainerArrayProxy(DataContainerArrayProxyTest::Key, DataContainerArrayProxy());
    QMap<QString, DataContainerProxy> dcMap = proxy.dataContainers;
    DREAM3D_REQUIRE_EQUAL(dcMap.size(), 1)
      DREAM3D_REQUIRE_EQUAL(dcMap.contains(DataContainerArrayProxyTest::DcProxyName), true)

      QMap<QString, AttributeMatrixProxy> amMap = dcMap.value(DataContainerArrayProxyTest::DcProxyName).attributeMatricies;
    DREAM3D_REQUIRE_EQUAL(amMap.size(), 2)
    DREAM3D_REQUIRE_EQUAL(amMap.contains(DataContainerArrayProxyTest::AmProxyName1), true)
      DREAM3D_REQUIRE_EQUAL(amMap.contains(DataContainerArrayProxyTest::AmProxyName2), true)

      QMap<QString, DataArrayProxy> daMap1 = amMap.value(DataContainerArrayProxyTest::AmProxyName1).dataArrays;
    DREAM3D_REQUIRE_EQUAL(daMap1.size(), 2)
      DREAM3D_REQUIRE_EQUAL(daMap1.contains(DataContainerArrayProxyTest::DaProxyName1), true)
      DREAM3D_REQUIRE_EQUAL(daMap1.contains(DataContainerArrayProxyTest::DaProxyName2), true)

    QMap<QString, DataArrayProxy> daMap2 = amMap.value(DataContainerArrayProxyTest::AmProxyName2).dataArrays;
    DREAM3D_REQUIRE_EQUAL(daMap2.size(), 1)
      DREAM3D_REQUIRE_EQUAL(daMap2.contains(DataContainerArrayProxyTest::DaProxyName3), true)
  }

  // DataArrayPath read test
  {
    DataArrayPath path = reader->readDataArrayPath(DataArrayPathTest::Key, DataArrayPath());
    DREAM3D_REQUIRE_EQUAL(path.isEmpty(), false)

      DREAM3D_REQUIRE_EQUAL(path.getDataContainerName(), DataArrayPathTest::DCName)
      DREAM3D_REQUIRE_EQUAL(path.getAttributeMatrixName(), DataArrayPathTest::AMName)
      DREAM3D_REQUIRE_EQUAL(path.getDataArrayName(), DataArrayPathTest::DAName)
  }

  // QVector<DataArrayPath> read test
  {
    QVector<DataArrayPath> paths = reader->readDataArrayPathVector(MultipleDataArrayPathTest::Key, QVector<DataArrayPath>());
    DREAM3D_REQUIRE_EQUAL(paths.size(), 2)

      DREAM3D_REQUIRE_EQUAL(paths[0].getDataContainerName(), MultipleDataArrayPathTest::DCName1)
      DREAM3D_REQUIRE_EQUAL(paths[0].getAttributeMatrixName(), MultipleDataArrayPathTest::AMName1)
      DREAM3D_REQUIRE_EQUAL(paths[0].getDataArrayName(), MultipleDataArrayPathTest::DAName1)

      DREAM3D_REQUIRE_EQUAL(paths[1].getDataContainerName(), MultipleDataArrayPathTest::DCName2)
      DREAM3D_REQUIRE_EQUAL(paths[1].getAttributeMatrixName(), MultipleDataArrayPathTest::AMName2)
      DREAM3D_REQUIRE_EQUAL(paths[1].getDataArrayName(), MultipleDataArrayPathTest::DAName2)
  }

  // DynamicTableData read test
  {
    DynamicTableData data = reader->readDynamicTableData(DynamicTableDataTest::Key, DynamicTableData());
    DREAM3D_REQUIRE_EQUAL(data.isEmpty(), false)

      QStringList rowHeaders = data.getRowHeaders();
    DREAM3D_REQUIRE_EQUAL(rowHeaders.size(), 6)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[0], DynamicTableDataTest::RowHeader1)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[1], DynamicTableDataTest::RowHeader2)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[2], DynamicTableDataTest::RowHeader3)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[3], DynamicTableDataTest::RowHeader4)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[4], DynamicTableDataTest::RowHeader5)
      DREAM3D_REQUIRE_EQUAL(rowHeaders[5], DynamicTableDataTest::RowHeader6)

      QStringList colHeaders = data.getColHeaders();
    DREAM3D_REQUIRE_EQUAL(colHeaders.size(), 8)
      DREAM3D_REQUIRE_EQUAL(colHeaders[0], DynamicTableDataTest::ColumnHeader1)
      DREAM3D_REQUIRE_EQUAL(colHeaders[1], DynamicTableDataTest::ColumnHeader2)
      DREAM3D_REQUIRE_EQUAL(colHeaders[2], DynamicTableDataTest::ColumnHeader3)
      DREAM3D_REQUIRE_EQUAL(colHeaders[3], DynamicTableDataTest::ColumnHeader4)
      DREAM3D_REQUIRE_EQUAL(colHeaders[4], DynamicTableDataTest::ColumnHeader5)
      DREAM3D_REQUIRE_EQUAL(colHeaders[5], DynamicTableDataTest::ColumnHeader6)
      DREAM3D_REQUIRE_EQUAL(colHeaders[6], DynamicTableDataTest::ColumnHeader7)
      DREAM3D_REQUIRE_EQUAL(colHeaders[7], DynamicTableDataTest::ColumnHeader8)

      std::vector<std::vector<double> > tableData = data.getTableData();
    DREAM3D_REQUIRE_EQUAL(tableData.size(), rowHeaders.size())
      foreach(std::vector<double> vector, tableData)
    {
      DREAM3D_REQUIRE_EQUAL(vector.size(), colHeaders.size())
      foreach(double i, vector)
      {
        DREAM3D_REQUIRE_EQUAL(vector[i], 0)
      }
    }
  }

  err = reader->closeFilterGroup();
  DREAM3D_REQUIRE_EQUAL(err, 0)

  return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void loadFilterPlugins()
{
  // Register all the filters including trying to load those from Plugins
  FilterManager* fm = FilterManager::Instance();
  DREAM3DPluginLoader::LoadPluginFilters(fm);

  // Send progress messages from PipelineBuilder to this object for display
  QMetaObjectUtilities::RegisterMetaTypes();
}


// -----------------------------------------------------------------------------
//  Use test framework
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Instantiate the QCoreApplication that we need to get the current path and load plugins.
  QCoreApplication app(argc, argv);
  QCoreApplication::setOrganizationName("BlueQuartz Software");
  QCoreApplication::setOrganizationDomain("bluequartz.net");
  QCoreApplication::setApplicationName("FilterParametersRWTest");

  int err = EXIT_SUCCESS;
  DREAM3D_REGISTER_TEST(loadFilterPlugins());

  DREAM3D_REGISTER_TEST(TestJsonWriter())
  DREAM3D_REGISTER_TEST(TestJsonReader())

  DREAM3D_REGISTER_TEST(RemoveTestFiles())
  PRINT_TEST_SUMMARY();

  return err;
}

