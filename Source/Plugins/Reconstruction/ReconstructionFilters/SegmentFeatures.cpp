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


#include "SegmentFeatures.h"

#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersReader.h"
#include "DREAM3DLib/FilterParameters/AbstractFilterParametersWriter.h"

#include "Reconstruction/ReconstructionConstants.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
SegmentFeatures::SegmentFeatures() :
  AbstractFilter(),
  m_DataContainerName(DREAM3D::Defaults::DataContainerName)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
SegmentFeatures::~SegmentFeatures()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SegmentFeatures::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int SegmentFeatures::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  DREAM3D_FILTER_WRITE_PARAMETER(FilterVersion)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SegmentFeatures::dataCheck()
{
  setErrorCondition(0);

  getDataContainerArray()->getPrereqGeometryFromDataContainer<ImageGeom, AbstractFilter>(this, getDataContainerName());
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SegmentFeatures::preflight()
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
int64_t SegmentFeatures::getSeed(int32_t gnum)
{
  return -1;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool SegmentFeatures::determineGrouping(int64_t referencepoint, int64_t neighborpoint, int32_t gnum)
{
  return false;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void SegmentFeatures::execute()
{
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getDataContainerName());

  size_t udims[3] = { 0, 0, 0 };
  m->getGeometryAs<ImageGeom>()->getDimensions(udims);
#if (CMP_SIZEOF_SIZE_T == 4)
  typedef int32_t DimType;
#else
  typedef int64_t DimType;
#endif
  DimType dims[3] =
  { static_cast<DimType>(udims[0]), static_cast<DimType>(udims[1]), static_cast<DimType>(udims[2]), };

  int32_t gnum = 1;
  int64_t seed = 0;
  DimType neighbor = 0;
  bool good = 0;
  DimType col = 0, row = 0, plane = 0;
  size_t size = 0;
  size_t initialVoxelsListSize = 100000;
  std::vector<int64_t> voxelslist(initialVoxelsListSize, -1);
  DimType neighpoints[6] = { 0, 0, 0, 0, 0, 0 };
  neighpoints[0] = -(dims[0] * dims[1]);
  neighpoints[1] = -dims[0];
  neighpoints[2] = -1;
  neighpoints[3] = 1;
  neighpoints[4] = dims[0];
  neighpoints[5] = (dims[0] * dims[1]);

  while (seed >= 0)
  {
    seed = getSeed(gnum);
    if (seed >= 0)
    {
      size = 0;
      voxelslist[size] = seed;
      size++;
      while (size > 0)
      {
        DimType currentpoint = voxelslist[size - 1];
        size -= 1;
        col = currentpoint % dims[0];
        row = (currentpoint / dims[0]) % dims[1];
        plane = currentpoint / (dims[0] * dims[1]);
        for (int32_t i = 0; i < 6; i++)
        {
          good = true;
          neighbor = currentpoint + neighpoints[i];
          if (i == 0 && plane == 0) { good = false; }
          if (i == 5 && plane == (dims[2] - 1)) { good = false; }
          if (i == 1 && row == 0) { good = false; }
          if (i == 4 && row == (dims[1] - 1)) { good = false; }
          if (i == 2 && col == 0) { good = false; }
          if (i == 3 && col == (dims[0] - 1)) { good = false; }
          if (good == true)
          {
            if (determineGrouping(currentpoint, neighbor, gnum) == true)
            {
              voxelslist[size] = neighbor;
              size++;
              if (size >= voxelslist.size())
              {
                size = voxelslist.size();
                voxelslist.resize(size + initialVoxelsListSize);
                for (std::vector<int64_t>::size_type j = size; j < voxelslist.size(); ++j) { voxelslist[j] = -1; }
              }
            }
          }
        }
      }
      voxelslist.clear();
      voxelslist.resize(initialVoxelsListSize, -1);
      gnum++;
      QString ss = QObject::tr("Total Features: %1").arg(gnum);
      if (gnum % 100 == 0) { notifyStatusMessage(getMessagePrefix(), getHumanLabel(), ss); }
    }
    if(getCancel()) { break; }
  }

  // If there is an error set this to something negative and also set a message
  notifyStatusMessage(getHumanLabel(), "Completed");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer SegmentFeatures::newFilterInstance(bool copyFilterParameters)
{
  SegmentFeatures::Pointer filter = SegmentFeatures::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SegmentFeatures::getCompiledLibraryName()
{ return ReconstructionConstants::ReconstructionBaseName; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SegmentFeatures::getGroupName()
{ return DREAM3D::FilterGroups::ReconstructionFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SegmentFeatures::getSubGroupName()
{ return DREAM3D::FilterSubGroups::SegmentationFilters; }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString SegmentFeatures::getHumanLabel()
{ return "Segment Features"; }
