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
FloatArrayType::Pointer DetermineStitching::FindGlobalOrigins(size_t totalPoints, QVector<size_t> udims, float sampleOrigin[3],  float voxelResolution[3], QVector<ImageProcessing::DefaultPixelType* > dataArrayList, QVector<float> xGlobCoordsList, QVector<float> yGlobCoordsList, QVector<size_t> xTileList, QVector<size_t> yTileList)
{

    QVector<size_t> cDims(1, 2);  // a dimension for the xvalues and one for the y values
    QVector<size_t> tDims(1);

    tDims[0] = xTileList.size();

    FloatArrayType::Pointer xyStitchedGlobalListPtr = FloatArrayType::CreateArray(tDims, cDims, "xyGlobalList");



    size_t numXtiles = FindMaxValue(xTileList) + 1;
    size_t numYtiles = FindMaxValue(yTileList) + 1;

    QVector<size_t> combIndexList(xTileList.size());

    combIndexList = ReturnIndexForCombOrder(xTileList, yTileList, numXtiles, numYtiles);
    size_t combIndex;

    ImageProcessing::UInt8ImageType* currentImage;
    ImageProcessing::UInt8ImageType* leftImage;
    ImageProcessing::UInt8ImageType* aboveImage;
    ImageProcessing::ImportUInt8FilterType::Pointer importFilter;
    ImageProcessing::ImportUInt8FilterType::Pointer importFilter2;
    std::vector<float> cropSpecsIm1Im2(12, 0);

    xyStitchedGlobalListPtr->setValue(0, 0);
    xyStitchedGlobalListPtr->setValue(1, 0);





    for (size_t i=1; i<combIndexList.size(); i++)
    {
       if (i < numXtiles)
       {

           //get filter to convert m_RawImageData to itk::image
           //if the image is in the top row of images, we need only the image to the left
           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i]]);
           currentImage = importFilter->GetOutput();
           importFilter2 = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i-1]]);
           leftImage = importFilter2->GetOutput();

           //////// TESTING ///////////
           typedef itk::ImageFileWriter< ImageProcessing::UInt8ImageType > WriterType;
           WriterType::Pointer writer = WriterType::New();
           writer->SetFileName( "/Users/megnashah/Desktop/fixedImageWindow.tiff");
           writer->SetInput( leftImage );
           writer->Update();
           writer->SetReleaseDataFlag(true);
           writer->SetFileName( "/Users/megnashah/Desktop/CurrentImageWindow.tiff");
           writer->SetInput( currentImage );
           writer->Update();




           cropSpecsIm1Im2[0] = 1200; //(47369.25 - 47113.25)/0.2079867; //xGlobCoordsList[1] - xyStitchedGlobalListPtr->getValue(2*(i-1)); //left image X Origin
           cropSpecsIm1Im2[1] = 0; //left image Y Origin
           cropSpecsIm1Im2[2] = 0; //left image Z Origin
           cropSpecsIm1Im2[3] = 0; //current image X Origin
           cropSpecsIm1Im2[4] = 0; //current image Y Origin
           cropSpecsIm1Im2[5] = 0; //current image Z Origin

           cropSpecsIm1Im2[6] = udims[0] - cropSpecsIm1Im2[0]; //left image X Size
           cropSpecsIm1Im2[7] = udims[1]; //left image Y Size
           cropSpecsIm1Im2[8] = 1; //left image Z Size
           cropSpecsIm1Im2[9] = udims[0] - cropSpecsIm1Im2[0]; //current image X Size
           cropSpecsIm1Im2[10] = udims[1]; //current image Y Size
           cropSpecsIm1Im2[11] = 1; //current image Z Size

           CropAndCrossCorrelate(cropSpecsIm1Im2, currentImage, leftImage);

           //////TESTING////////


           cropSpecsIm1Im2[0] = xGlobCoordsList[combIndexList[i]] - xyStitchedGlobalListPtr->getValue(2*(i-1)); //left image X Origin
           cropSpecsIm1Im2[1] = 0; //left image Y Origin
           cropSpecsIm1Im2[2] = 0; //left image Z Origin
           cropSpecsIm1Im2[3] = 0; //current image X Origin
           cropSpecsIm1Im2[4] = 0; //current image Y Origin
           cropSpecsIm1Im2[5] = 0; //current image Z Origin

           cropSpecsIm1Im2[6] = udims[0] - cropSpecsIm1Im2[0]; //left image X Size
           cropSpecsIm1Im2[7] = udims[1]; //left image Y Size
           cropSpecsIm1Im2[8] = 1; //left image Z Size
           cropSpecsIm1Im2[9] = udims[0] - cropSpecsIm1Im2[0]; //current image X Size
           cropSpecsIm1Im2[10] = udims[1]; //current image Y Size
           cropSpecsIm1Im2[11] = 1; //current image Z Size





       }

       if (i%numXtiles == 0)
       {

           //get filter to convert m_RawImageData to itk::image
           //if the image is in the first (left most) column of images, we only need the top image
           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i]]);
           currentImage = importFilter->GetOutput();
           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i-numXtiles]]);
           aboveImage = importFilter->GetOutput();

           cropSpecsIm1Im2[0] = 0; //top image X Origin
           cropSpecsIm1Im2[1] = yGlobCoordsList[combIndexList[i]] - xyStitchedGlobalListPtr->getValue(2*(i-numXtiles)+1);; //top image Y Origin
           cropSpecsIm1Im2[2] = 0; //top image Z Origin
           cropSpecsIm1Im2[3] = 0; //current image X Origin
           cropSpecsIm1Im2[4] = 0; //current image Y Origin
           cropSpecsIm1Im2[5] = 0; //current image Z Origin

           cropSpecsIm1Im2[6] = udims[0]; //top image X Size
           cropSpecsIm1Im2[7] = udims[1] - cropSpecsIm1Im2[1]; //top image Y Size
           cropSpecsIm1Im2[8] = 1; //top image Z Size
           cropSpecsIm1Im2[9] = udims[0]; //current image X Size
           cropSpecsIm1Im2[10] = udims[1] - cropSpecsIm1Im2[1]; //current image Y Size
           cropSpecsIm1Im2[11] = 1; //current image Z Size






       }

       else
       {

           //get filter to convert m_RawImageData to itk::image
           //for all other images, we need to match to the top and the left
           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i]]);
           currentImage = importFilter->GetOutput();
           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i-numXtiles]]);
           aboveImage = importFilter->GetOutput();

           cropSpecsIm1Im2[0] = 0; //top image X Origin
           cropSpecsIm1Im2[1] = yGlobCoordsList[combIndexList[i]] - xyStitchedGlobalListPtr->getValue(2*(i-numXtiles)+1);; //top image Y Origin
           cropSpecsIm1Im2[2] = 0; //top image Z Origin
           cropSpecsIm1Im2[3] = 0; //current image X Origin
           cropSpecsIm1Im2[4] = 0; //current image Y Origin
           cropSpecsIm1Im2[5] = 0; //current image Z Origin

           cropSpecsIm1Im2[6] = udims[0]; //top image X Size
           cropSpecsIm1Im2[7] = udims[1] - cropSpecsIm1Im2[1]; //top image Y Size
           cropSpecsIm1Im2[8] = 1; //top image Z Size
           cropSpecsIm1Im2[9] = udims[0]; //current image X Size
           cropSpecsIm1Im2[10] = udims[1] - cropSpecsIm1Im2[1]; //current image Y Size
           cropSpecsIm1Im2[11] = 1; //current image Z Size


           importFilter = ITKUtilitiesType::Dream3DtoITKImportFilterDataArray<ImageProcessing::DefaultPixelType>(totalPoints, udims, sampleOrigin, voxelResolution, dataArrayList[combIndexList[i-1]]);
           leftImage = importFilter->GetOutput();

           cropSpecsIm1Im2[0] = xGlobCoordsList[combIndexList[i]] - xyStitchedGlobalListPtr->getValue(2*(i-1)); //left image X Origin
           cropSpecsIm1Im2[1] = 0; //left image Y Origin
           cropSpecsIm1Im2[2] = 0; //left image Z Origin
           cropSpecsIm1Im2[3] = 0; //current image X Origin
           cropSpecsIm1Im2[4] = 0; //current image Y Origin
           cropSpecsIm1Im2[5] = 0; //current image Z Origin

           cropSpecsIm1Im2[6] = udims[0] - cropSpecsIm1Im2[0]; //left image X Size
           cropSpecsIm1Im2[7] = udims[1]; //left image Y Size
           cropSpecsIm1Im2[8] = 1; //left image Z Size
           cropSpecsIm1Im2[9] = udims[0] - cropSpecsIm1Im2[0]; //current image X Size
           cropSpecsIm1Im2[10] = udims[1]; //current image Y Size
           cropSpecsIm1Im2[11] = 1; //current image Z Size

       }

    }


    return xyStitchedGlobalListPtr;

}







std::vector<float> DetermineStitching::CropAndCrossCorrelate(std::vector<float> cropSpecsIm1Im2,  ImageProcessing::UInt8ImageType* currentImage,  ImageProcessing::UInt8ImageType* fixedImage)
{

    std::vector<float> newXYOrigin(2, 0);
//    //get image from filter
//        const ImageProcessing::UInt8ImageType* inputImage = importFilter->GetOutput();
//        ImageProcessing::UInt8ImageType::RegionType filterRegion = inputImage->GetBufferedRegion();
//        ImageProcessing::UInt8ConstIteratorType it(inputImage, filterRegion);




//        std::cout << "Image largest region: " << filterRegion.GetSize() << std::endl;


        //////FIRST IMAGE CROP
        ImageProcessing::UInt8ImageType::RegionType cropRegion;

        cropRegion.SetIndex(0, cropSpecsIm1Im2[0]);
        cropRegion.SetIndex(1, cropSpecsIm1Im2[1]);
        cropRegion.SetIndex(2, cropSpecsIm1Im2[2]);

        cropRegion.SetSize(0, cropSpecsIm1Im2[6]);
        cropRegion.SetSize(1, cropSpecsIm1Im2[7]);
        cropRegion.SetSize(2, cropSpecsIm1Im2[8]);

        typedef itk::ExtractImageFilter< ImageProcessing::UInt8ImageType, ImageProcessing::UInt8ImageType > exImFilterType;
        exImFilterType::Pointer exImfilter = exImFilterType::New();
        exImfilter->SetExtractionRegion(cropRegion);
        exImfilter->SetInput(fixedImage);
      #if ITK_VERSION_MAJOR >= 4
        exImfilter->SetDirectionCollapseToIdentity(); // This is required.
      #endif
        exImfilter->Update();

        ImageProcessing::UInt8ImageType* fixedImageWindow = exImfilter->GetOutput();







        /////////////////////SECOND IMAGE CROP
        ImageProcessing::UInt8ImageType::RegionType cropRegion2;
        cropRegion2.SetIndex(0, cropSpecsIm1Im2[3]);
        cropRegion2.SetIndex(1, cropSpecsIm1Im2[4]);
        cropRegion2.SetIndex(2, cropSpecsIm1Im2[5]);

        cropRegion2.SetSize(0, cropSpecsIm1Im2[9]);
        cropRegion2.SetSize(1, cropSpecsIm1Im2[10]);
        cropRegion2.SetSize(2, cropSpecsIm1Im2[11]);

        exImFilterType::Pointer exImfilter2 = exImFilterType::New();
        exImfilter2->SetExtractionRegion(cropRegion2);
        exImfilter2->SetInput(currentImage);
        #if ITK_VERSION_MAJOR >= 4
          exImfilter2->SetDirectionCollapseToIdentity(); // This is required.
        #endif
          exImfilter2->Update();
          ImageProcessing::UInt8ImageType* currentImageWindow = exImfilter2->GetOutput();

        /////WRITING THE IMAGES FOR TESTING

        typedef itk::ImageFileWriter< ImageProcessing::UInt8ImageType > WriterType;

        WriterType::Pointer writer = WriterType::New();
        writer->SetFileName( "/Users/megnashah/Desktop/fixedImageWindow.tiff");
        writer->SetInput( fixedImageWindow );
        writer->Update();

        WriterType::Pointer writer2 = WriterType::New();
        writer2->SetFileName( "/Users/megnashah/Desktop/CurrentImageWindow.tiff");
        writer2->SetInput( currentImageWindow );
        writer2->Update();



        typedef itk::MaskedFFTNormalizedCorrelationImageFilter< ImageProcessing::DefaultImageType, ImageProcessing::FloatImageType, ImageProcessing::DefaultImageType > XCFilterType;
        XCFilterType::Pointer xCorrFilter = XCFilterType::New();
        fixedImageWindow->DisconnectPipeline();
        fixedImageWindow->FillBuffer(2);

        ImageProcessing::UInt8ImageType::RegionType resetOrigin;
        resetOrigin.SetIndex(0, 0);
        resetOrigin.SetIndex(1, 0);
        resetOrigin.SetIndex(2, 0);

        resetOrigin.SetSize(0, fixedImageWindow->GetLargestPossibleRegion().GetSize()[0]);
        resetOrigin.SetSize(1, fixedImageWindow->GetLargestPossibleRegion().GetSize()[1]);
        resetOrigin.SetSize(2, fixedImageWindow->GetLargestPossibleRegion().GetSize()[2]);

        fixedImageWindow->SetLargestPossibleRegion(resetOrigin);
        currentImageWindow->DisconnectPipeline();

        xCorrFilter->SetFixedImage(fixedImageWindow);
        xCorrFilter->SetMovingImage(currentImageWindow);

        std::cout << "fixed" << fixedImageWindow->GetLargestPossibleRegion().GetSize()<< std::endl;
        std::cout << "fixed" << fixedImageWindow->GetLargestPossibleRegion().GetIndex()<< std::endl;
        std::cout << "fixed" << fixedImageWindow->GetBufferedRegion()<< std::endl;
        std::cout << "current" << currentImageWindow->GetLargestPossibleRegion().GetSize()<< std::endl;
        std::cout << "current" << currentImageWindow->GetLargestPossibleRegion().GetIndex()<< std::endl;
        std::cout << "current" << currentImageWindow->GetBufferedRegion()<< std::endl;
        xCorrFilter->Update();
//        xCorrFilter->SetRequiredFractionOfOverlappingPixels(1);
        ImageProcessing::FloatImageType* xcoutputImage = xCorrFilter->GetOutput();

        typedef itk::ImageFileWriter< ImageProcessing::FloatImageType > nWriterType;
        nWriterType::Pointer writer3 = nWriterType::New();
        writer3->SetFileName( "/Users/megnashah/Desktop/imageXC.tiff");
        writer3->SetInput( xcoutputImage );
        writer3->Update();
    return newXYOrigin;
}

//Finds the max value in a vector ... possibly redunant since it is one line ...
size_t DetermineStitching::FindMaxValue(QVector<size_t> inputVector)
{

    QVector<size_t>::iterator it = std::max_element(inputVector.begin(), inputVector.end());

    return *it;

}



//This helper function takes the tile list and creates a new vector that orders the tiles as though they are in comb order. So a tile set collected
//in a comb fashion (along the rows first) will have the values in the new vector match the index. This is a helper so that we can always stitch the
//tiles the same way regardless of how they were collected.


QVector<size_t> DetermineStitching::ReturnIndexForCombOrder(QVector<size_t> xTileList, QVector<size_t> yTileList, size_t numXtiles, size_t numYtiles)
{

    QVector<size_t> newIndices(xTileList.size(), 0);
    size_t count;

    for (size_t iter = 0; iter < xTileList.size(); iter++)
    {
        count = 0;
        for (size_t j=0; j<numYtiles; j++)
        {
            for (size_t i=0; i<numXtiles; i++)
            {
                if (xTileList[iter] == i && yTileList[iter] == j)
                {
                    newIndices[iter] = count;
                }
                count ++;
            }
        }
    }

    return newIndices;

}
