/* ============================================================================
 * Copyright (c) 2011 Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2011 Dr. Michael A. Groeber (US Air Force Research Laboratories)
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


#include "LosAlamosFFTReader.h"

#include <iostream>
#include <cstring>
#include "MXA/Utilities/MXAFileInfo.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LosAlamosFFTReader::LosAlamosFFTReader() :
  AbstractFilter(),
  m_Stress11ArrayName(DREAM3D::CellData::Stress11),
  m_Stress22ArrayName(DREAM3D::CellData::Stress22),
  m_Stress33ArrayName(DREAM3D::CellData::Stress33),
  m_Stress23ArrayName(DREAM3D::CellData::Stress23),
  m_Stress31ArrayName(DREAM3D::CellData::Stress31),
  m_Stress12ArrayName(DREAM3D::CellData::Stress12),
  m_Strain11ArrayName(DREAM3D::CellData::Strain11),
  m_Strain22ArrayName(DREAM3D::CellData::Strain22),
  m_Strain33ArrayName(DREAM3D::CellData::Strain33),
  m_Strain23ArrayName(DREAM3D::CellData::Strain23),
  m_Strain31ArrayName(DREAM3D::CellData::Strain31),
  m_Strain12ArrayName(DREAM3D::CellData::Strain12),
  m_StressArrayName(DREAM3D::CellData::Stress),
  m_StrainArrayName(DREAM3D::CellData::Strain),
  m_EEDArrayName(DREAM3D::CellData::EED),
  m_MaxPrincipalStressArrayName(DREAM3D::CellData::MaxPrincipalStress),
  m_MinPrincipalStressArrayName(DREAM3D::CellData::MinPrincipalStress),
  m_FieldsFile(""),
  m_RStatsFile(""),
  m_Stress11(NULL),
  m_Stress22(NULL),
  m_Stress33(NULL),
  m_Stress23(NULL),
  m_Stress31(NULL),
  m_Stress12(NULL),
  m_Strain11(NULL),
  m_Strain22(NULL),
  m_Strain33(NULL),
  m_Strain23(NULL),
  m_Strain31(NULL),
  m_Strain12(NULL),
  m_Stress(NULL),
  m_Strain(NULL),
  m_EED(NULL),
  m_MaxPrincipalStress(NULL),
  m_MinPrincipalStress(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LosAlamosFFTReader::~LosAlamosFFTReader()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LosAlamosFFTReader::setupFilterParameters()
{
  std::vector<FilterParameter::Pointer> parameters;
  {
    FilterParameter::Pointer option = FilterParameter::New();
    option->setHumanLabel("Fields File");
    option->setPropertyName("FieldsFile");
    option->setWidgetType(FilterParameter::InputFileWidget);
    option->setValueType("string");
    option->setFileExtension("*.out");
    parameters.push_back(option);
  }
  {
    FilterParameter::Pointer option = FilterParameter::New();
    option->setHumanLabel("R-Stats File");
    option->setPropertyName("RStatsFile");
    option->setWidgetType(FilterParameter::InputFileWidget);
    option->setValueType("string");
    option->setFileExtension("*.txt");
    parameters.push_back(option);
  }
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LosAlamosFFTReader::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  /* Code to read the values goes between these statements */
/* FILTER_WIDGETCODEGEN_AUTO_GENERATED_CODE BEGIN*/
  setFieldsFile( reader->readValue( "FieldsFile", getFieldsFile()) );
  setRStatsFile( reader->readValue( "RStatsFile", getRStatsFile()) );
/* FILTER_WIDGETCODEGEN_AUTO_GENERATED_CODE END*/
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int LosAlamosFFTReader::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  writer->writeValue("FieldsFile", getFieldsFile() );
  writer->writeValue("RStatsFile", getRStatsFile() );
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LosAlamosFFTReader::dataCheck(bool preflight, size_t voxels, size_t fields, size_t ensembles)
{

  setErrorCondition(0);
  std::stringstream ss;
  VolumeDataContainer* m = getVolumeDataContainer();

  if (getFieldsFile().empty() == true)
  {
    ss << ClassName() << " needs the Fields File Set and it was not.";
    setErrorCondition(-387);
    addErrorMessage(getHumanLabel(), ss.str(), getErrorCondition());
  }
  else if (MXAFileInfo::exists(getFieldsFile()) == false)
  {
    ss << "Fields input file does not exist.";
    setErrorCondition(-388);
    addErrorMessage(getHumanLabel(), ss.str(), getErrorCondition());
  }

  if (getRStatsFile().empty() == true)
  {
    ss << ClassName() << " needs the R-Stats File Set and it was not.";
    setErrorCondition(-387);
    addErrorMessage(getHumanLabel(), ss.str(), getErrorCondition());
  }
  else if (MXAFileInfo::exists(getRStatsFile()) == false)
  {
    ss << "The input file does not exist.";
    setErrorCondition(-388);
    addErrorMessage(getHumanLabel(), ss.str(), getErrorCondition());
  }

  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress11, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress22, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress33, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress23, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress31, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress12, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain11, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain22, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain33, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain23, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain31, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain12, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Stress, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, Strain, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, EED, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, MaxPrincipalStress, float, FloatArrayType, 0, voxels, 1)
  CREATE_NON_PREREQ_DATA(m, DREAM3D, CellData, MinPrincipalStress, float, FloatArrayType, 0, voxels, 1)
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LosAlamosFFTReader::preflight()
{
  dataCheck(true, 1, 1, 1);
}

float LosAlamosFFTReader::sciToF(char* buff)
{
    char* p = strchr(buff, 'E');
    if (p==NULL)
    {
        p = strchr(buff, 'e');
    }
    if(p==NULL)
    {
        return NULL;
    }
    int index = p-buff;

    //char base[index];
    char base[8];
    strncpy(base, buff, std::min(index,8));

    //char exp[strlen(buff)-(index+1)];
    char exp[8];
    strncpy(exp, buff+index+1, strlen(buff)-(index+1));

    return atof(base)*pow(10,atoi(exp));
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void  LosAlamosFFTReader::execute()
{
  int err = 0;
  setErrorCondition(err);
  VolumeDataContainer* m = getVolumeDataContainer();
  if(NULL == m)
  {
    setErrorCondition(-999);
    notifyErrorMessage("The DataContainer Object was NULL", getErrorCondition());
    return;
  }
  dataCheck(false, m->getTotalPoints(), m->getNumFieldTuples(), m->getNumEnsembleTuples());
  if (getErrorCondition() < 0)
  {
    return;
  }

  int64_t totalPoints = m->getTotalPoints();


  //open file
    //const char *fileName = getFieldsFile();
    FILE *pFile;
    pFile = fopen(m_FieldsFile.c_str(), "rb");
    if (pFile==NULL)
    {
        notifyErrorMessage("unable to open file",-998);
    }

    //get length of file
    long lSize;
    fseek(pFile, 0, SEEK_END);
    lSize = ftell(pFile);
    rewind(pFile);

    //allocate enough memory to read whole file
    char* buffer;
    buffer = (char*) malloc(sizeof(char)*lSize);
    if (buffer == NULL)
    {
        notifyErrorMessage("unable to allocate memory for file",-998);
    }

    //load file into allocated memory
    size_t result;
    result = fread(buffer, 1, lSize, pFile);
    if (result != lSize)
    {
        notifyErrorMessage("unable to load file",-998);
    }

    for (int64_t i=0; i<totalPoints; i++)
    {
        m_Stress11[i]=0.0f;
        m_Stress22[i]=0.0f;
        m_Stress33[i]=0.0f;
        m_Stress23[i]=0.0f;
        m_Stress31[i]=0.0f;
        m_Stress12[i]=0.0f;
        m_Strain11[i]=0.0f;
        m_Strain22[i]=0.0f;
        m_Strain33[i]=0.0f;
        m_Strain23[i]=0.0f;
        m_Strain31[i]=0.0f;
        m_Strain12[i]=0.0f;
        m_Stress[i]=0.0f;
        m_Strain[i]=0.0f;
        m_EED[i]=0.0f;
        m_MaxPrincipalStress[i]=0.0f;
        m_MinPrincipalStress[i]=0.0f;
    }

    //break file into tokens

    //first token is side length
    char *token = strtok(buffer, " \n");
    //next token is # pts
    token = strtok(NULL, " \n");
    int64_t voxels=atoi(token);

    if(voxels!=totalPoints)
    {
        notifyErrorMessage("number of voxels doesn't match",-997);
    }

    bool dataStart=false;
    int i=0;
    int j=-1;
    while(token && i<totalPoints)
    {
        if(!dataStart)
        if(strcmp(token, "SLOC")==0)
        {
            dataStart=true;
        }

        if(dataStart)
        {
            switch(j)
            {
                case 0://?
                    {

                    }
                    break;

                case 1://?
                    {

                    }
                    break;

                case 2://?
                    {

                    }
                    break;

                case 3://X
                    {

                    }
                    break;

                case 4://Y
                    {

                    }
                    break;

                case 5://Z
                    {

                    }
                    break;

                case 6://phase
                    {

                    }
                    break;

                case 7://spin
                    {

                    }
                    break;

                case 8://strain 11
                    {
                        m_Strain11[i]=sciToF(token);
                    }
                    break;

                case 9://strain 22
                    {
                        m_Strain22[i]=sciToF(token);
                    }
                    break;

                case 10://strain 33
                    {
                        m_Strain33[i]=sciToF(token);
                    }
                    break;

                case 11://strain 23
                    {
                        m_Strain23[i]=sciToF(token);
                    }
                    break;

                case 12://strain 31
                    {
                        m_Strain31[i]=sciToF(token);
                    }
                    break;

                case 13://strain 12
                    {
                        m_Strain12[i]=sciToF(token);
                    }
                    break;

                case 14://stress 11
                    {
                        m_Stress11[i]=sciToF(token);
                    }
                    break;

                case 15://stress 22
                    {
                        m_Stress22[i]=sciToF(token);
                    }
                    break;

                case 16://stress 33
                    {
                        m_Stress33[i]=sciToF(token);
                    }
                    break;

                case 17://stress 23
                    {
                        m_Stress23[i]=sciToF(token);
                    }
                    break;

                case 18://stress 31
                    {
                        m_Stress31[i]=sciToF(token);
                    }
                    break;

                case 19://stress 12
                    {
                        m_Stress12[i]=sciToF(token);
                    }
                    break;
            }
            j++;
            if(j==20)
            {
                j=0;
                i++;
            }
        }
        token = strtok(NULL, " \n");
    }
    free(buffer);

    //const char *fileName = getFieldsFile();
    pFile = fopen(m_RStatsFile.c_str(), "rb");
    if (pFile==NULL)
    {
        notifyErrorMessage("unable to open file",-998);
    }

    //get length of file
    fseek(pFile, 0, SEEK_END);
    lSize = ftell(pFile);
    rewind(pFile);

    //allocate enough memory to read whole file
    buffer = (char*) malloc(sizeof(char)*lSize);
    if (buffer == NULL)
    {
        notifyErrorMessage("unable to allocate memory for file",-998);
    }

    //load file into allocated memory
    result = fread(buffer, 1, lSize, pFile);
    if (result != lSize)
    {
        notifyErrorMessage("unable to load file",-998);
    }

    //tokenize
    token = strtok(buffer, " \n");
    for (int i=0; i<6; i++)
    {
        token = strtok(NULL, " \n");
    }

    i=0;
    j=0;
    while(token && i<totalPoints)
    {
        switch(j)
        {
            case 0://strain
                {
                    m_Strain[i]=atof(token);
                }
                break;

            case 1://stress
                {
                    m_Stress[i]=atof(token);
                }
                break;

            case 2://eed
                {
                    m_EED[i]=atof(token);
                }
                break;

            case 3://maxprincipalstress
                {
                    m_MaxPrincipalStress[i]=atof(token);
                }
                break;

            case 4://minprincipalstress
                {
                    m_MinPrincipalStress[i]=atof(token);
                }
                break;

            case 5://phase
                {

                }
                break;
        }
        j++;
        if(j==6)
        {
            j=0;
            i++;
        }
        token = strtok(NULL, " \n");
    }




}
