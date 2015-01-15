/* ============================================================================
 * Copyright (c) 2014 Michael A. Jackson (BlueQuartz Software)
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
 * Neither the name of Michael A. Jackson, BlueQuartz Software nor the names of
 * its contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "DetermineStitching.h"

#include "itkMaskedFFTNormalizedCorrelationImageFilter.h"

#include "ImageProcessing/ImageProcessingFilters/ItkBridge.h"

#include "itkImage.h"
#include "itkImageFileWriter.h"


#include "ImageProcessing/ImageProcessingHelpers.hpp"






// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DetermineStitching::DetermineStitching()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DetermineStitching::~DetermineStitching()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
//FloatArrayType::Pointer DetermineStitching::FindGlobalOrigins(size_t totalPoints, QVector<size_t> udims, float sampleOrigin, float voxelResolution, int tileDims, QVector<size_t> dataArrayList)
FloatArrayType::Pointer DetermineStitching::FindGlobalOrigins(size_t totalPoints, QVector<size_t> udims, float sampleOrigin[3],  float voxelResolution[3], QVector<ImageProcessing::DefaultPixelType* > dataArrayList)
{

//    QVector<size_t> cDims(1, 2);
//    QVector<size_t> tDims(1);

//    //tDims[0] = am->getNumTuples();

//    FloatArrayType::Pointer xyGlobalListPtr = FloatArrayType::CreateArray(tDims, cDims, "xyGlobalList");

    //get filter to convert m_RawImageData to itk::image


    ImageProcessing::ImportUInt8FilterType::Pointer importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[0]);

    //get image from filter
    const ImageProcessing::UInt8ImageType* inputImage = importFilter->GetOutput();
    ImageProcessing::UInt8ImageType::RegionType filterRegion = inputImage->GetBufferedRegion();
    ImageProcessing::UInt8ConstIteratorType it(inputImage, filterRegion);

    typedef itk::MaskedFFTNormalizedCorrelationImageFilter< ImageProcessing::DefaultImageType, ImageProcessing::FloatImageType, ImageProcessing::DefaultImageType > XCFilterType;
    XCFilterType::Pointer xCorrFilter = XCFilterType::New();


    std::cout << "Image largest region: " << filterRegion.GetSize() << std::endl;

    ImageProcessing::UInt8ImageType::RegionType cropRegion;

    cropRegion.SetSize(0, 50);
    cropRegion.SetSize(1, 600);
    cropRegion.SetSize(2, 1);

    cropRegion.SetIndex(0, 0);
    cropRegion.SetIndex(1, 0);
    cropRegion.SetIndex(2, 0);

    typedef itk::ExtractImageFilter< ImageProcessing::UInt8ImageType, ImageProcessing::UInt8ImageType > exImFilterType;
    exImFilterType::Pointer exImfilter = exImFilterType::New();
    exImfilter->SetExtractionRegion(cropRegion);
    exImfilter->SetInput(inputImage);
  #if ITK_VERSION_MAJOR >= 4
    exImfilter->SetDirectionCollapseToIdentity(); // This is required.
  #endif
    exImfilter->Update();
    ImageProcessing::UInt8ImageType* outputImage = exImfilter->GetOutput();


    typedef itk::ImageFileWriter< ImageProcessing::UInt8ImageType > WriterType;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( "/Users/megnashah/Desktop/imageEXIm.tiff");
    writer->SetInput( outputImage );
    writer->Update();



    xCorrFilter->SetFixedImage(inputImage);
    xCorrFilter->SetMovingImage(outputImage);
    xCorrFilter->Update();
    xCorrFilter->SetRequiredFractionOfOverlappingPixels(1);
    ImageProcessing::FloatImageType* xcoutputImage = xCorrFilter->GetOutput();

    typedef itk::ImageFileWriter< ImageProcessing::FloatImageType > nWriterType;
    nWriterType::Pointer writer2 = nWriterType::New();
    writer2->SetFileName( "/Users/megnashah/Desktop/imageXC.tiff");
    writer2->SetInput( xcoutputImage );
    writer2->Update();

//    return xyGlobalListPtr;

}
