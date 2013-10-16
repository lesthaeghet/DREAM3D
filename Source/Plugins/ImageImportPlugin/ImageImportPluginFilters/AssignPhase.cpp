/* ============================================================================
 * Copyright (c) 2012 Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2012 Dr. Michael A. Groeber (US Air Force Research Laboratories)
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "AssignPhase.h"


#include "DREAM3DLib/Math/MatrixMath.h"
#include "EbsdLib/EbsdConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AssignPhase::AssignPhase() :
  AbstractFilter(),
  m_ImageDataArrayName(DREAM3D::CellData::ImageData),
  m_CellPhasesArrayName(DREAM3D::CellData::Phases),
  m_ImageData(NULL),
  m_CellPhases(NULL),
  m_Phase(NULL)
{
  m_GrayMin = 0;
  m_GrayMax = 255;

  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AssignPhase::~AssignPhase()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AssignPhase::setupFilterParameters()
{
  FilterParameterVector parameters;
  {
    ChoiceFilterParameter::Pointer option = ChoiceFilterParameter::New();
    option->setHumanLabel("Phase");
    option->setPropertyName("Phase");
    option->setWidgetType(FilterParameter::ChoiceWidget);
    option->setValueType("unsigned int");
    std::vector<std::string> choices;
    choices.push_back("Hexagonal High");
    choices.push_back("Cubic High");
    choices.push_back("Hexagonal Low");
    choices.push_back("Cubic Low");
    choices.push_back("Triclinic");
    choices.push_back("Monoclinic");
    choices.push_back("OrthoRhombic");
    choices.push_back("Tetragonal Low");
    choices.push_back("Tetragonal High");
    choices.push_back("Trigonal Low");
    choices.push_back("Trigonal High");
    option->setChoices(choices);
    parameters.push_back(option);
  }
  {
    FilterParameter::Pointer option = FilterParameter::New();
    option->setHumanLabel("Gray Value Floor");
    option->setPropertyName("GrayMin");
    option->setWidgetType(FilterParameter::IntWidget);
    option->setValueType("int");
    parameters.push_back(option);
  }
  {
    FilterParameter::Pointer option = FilterParameter::New();
    option->setHumanLabel("Gray Value Ceiling");
    option->setPropertyName("GrayMax");
    option->setWidgetType(FilterParameter::IntWidget);
    option->setValueType("int");
    parameters.push_back(option);
  }

  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AssignPhase::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  /* Code to read the values goes between these statements */
  /* FILTER_WIDGETCODEGEN_AUTO_GENERATED_CODE BEGIN*/
  setPhase( reader->readValue("Phase", getPhase() ) );
  setGrayMin( reader->readValue("GrayMin", getGrayMin() ) );
  setGrayMax( reader->readValue("GrayMax", getGrayMax() ) );
  /* FILTER_WIDGETCODEGEN_AUTO_GENERATED_CODE END*/
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int AssignPhase::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  /* Place code that will write the inputs values into a file. reference the
   AbstractFilterParametersWriter class for the proper API to use. */
  writer->writeValue("Phase", getPhase() );
  writer->writeValue("GrayMin", getGrayMin() );
  writer->writeValue("GrayMax", getGrayMax() );
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AssignPhase::dataCheck(bool preflight, size_t voxels, size_t fields, size_t ensembles)
{
  setErrorCondition(0);
  std::stringstream ss;
  VolumeDataContainer* m = getVolumeDataContainer();
  if (NULL == m)
  {
    std::stringstream ss;
    ss << getHumanLabel() << "The VolumeDataContainer was NULL and this is NOT allowed. There is an error in the programming. Please contact the developers";
    setErrorCondition(-1);
    addErrorMessage(getHumanLabel(), ss.str(), -1);
    return;
  }

  if(m_GrayMax<m_GrayMin)
  {
      std::stringstream ss;
      ss << getHumanLabel() << "The maximum value must be greater than or equal to the minimum value";
      setErrorCondition(-1);
      addErrorMessage(getHumanLabel(), ss.str(), -1);
      return;
  }
  if(m_GrayMin<0 || m_GrayMax<0)
  {
      std::stringstream ss;
      ss << getHumanLabel() << "The gray levels must be greater than or equal to 0";
      setErrorCondition(-1);
      addErrorMessage(getHumanLabel(), ss.str(), -1);
      return;
  }
  if(m_GrayMin>255 || m_GrayMax>255)
  {
      std::stringstream ss;
      ss << getHumanLabel() << "The gray levels must be less than or equal to 255";
      setErrorCondition(-1);
      addErrorMessage(getHumanLabel(), ss.str(), -1);
      return;
  }

  //needs image data and phases
  GET_PREREQ_DATA(m, DREAM3D, CellData, CellPhases, -301, int32_t, Int32ArrayType,  voxels, 1)
  GET_PREREQ_DATA(m, DREAM3D, CellData, ImageData, -302, uint8_t, UInt8ArrayType, voxels, 1)
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AssignPhase::preflight()
{
  /* Place code here that sanity checks input arrays and input values. Look at some
  * of the other DREAM3DLib/Filters/.cpp files for sample codes */
  dataCheck(true, 1, 1, 1);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void AssignPhase::execute()
{
  int err = 0;
  std::stringstream ss;
  setErrorCondition(err);
  VolumeDataContainer* m = getVolumeDataContainer();
  if(NULL == m)
  {
    setErrorCondition(-999);
    notifyErrorMessage("The Voxel DataContainer Object was NULL", -999);
    return;
  }
  int64_t totalPoints = m->getTotalPoints();
  size_t totalFields = m->getNumFieldTuples();
  size_t totalEnsembles = m->getNumEnsembleTuples();
  dataCheck(false, totalPoints, totalFields, totalEnsembles);
  if (getErrorCondition() < 0)
  {
    return;
  }

  UInt8ArrayType::Pointer notSupported = UInt8ArrayType::CreateArray(13, 1, "NotSupportedArray");
  notSupported->initializeWithZeros();

  unsigned int assignedPhase=Ebsd::CrystalStructure::UnknownCrystalStructure;
  switch(m_Phase)
  {
      case 0:
          {
              assignedPhase=Ebsd::CrystalStructure::Hexagonal_High;
          }
          break;

      case 1:
          {
              assignedPhase=Ebsd::CrystalStructure::Cubic_High;
          }
          break;

      case 2:
          {
              assignedPhase=Ebsd::CrystalStructure::Hexagonal_Low;
          }
          break;

      case 3:
          {
              assignedPhase=Ebsd::CrystalStructure::Cubic_Low;
          }
          break;

      case 4:
          {
              assignedPhase=Ebsd::CrystalStructure::Triclinic;
          }
          break;

      case 5:
          {
              assignedPhase=Ebsd::CrystalStructure::Monoclinic;
          }
          break;

      case 6:
          {
              assignedPhase=Ebsd::CrystalStructure::OrthoRhombic;
          }
          break;

      case 7:
          {
              assignedPhase=Ebsd::CrystalStructure::Tetragonal_Low;
          }
          break;

      case 8:
          {
              assignedPhase=Ebsd::CrystalStructure::Tetragonal_High;
          }
          break;

      case 9:
          {
              assignedPhase=Ebsd::CrystalStructure::Trigonal_Low;
          }
          break;

      case 10:
          {
              assignedPhase=Ebsd::CrystalStructure::Trigonal_High;
          }
          break;
  }

  // Write the Misorientation Coloring Cell Data
  for (int64_t i = 0; i < totalPoints; i++)
  {
      if(m_ImageData[i]<=m_GrayMax && m_ImageData[i]>=m_GrayMin)
      {
          m_CellPhases[i]=assignedPhase;
      }
  }

  /* Let the GUI know we are done with this filter */
  notifyStatusMessage("Complete");

}
