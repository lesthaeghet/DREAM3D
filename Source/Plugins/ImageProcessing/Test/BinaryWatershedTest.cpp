
#include <iostream>


#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryImageToLabelMapFilter.h"
#include "itkImageFileReader.h"


int main(int argc, char *argv[])
{

  std::cout << "BinaryWatershedTest Starting" << std::endl;
/////////////////TEST//////////////


  typedef itk::Image<unsigned char, 2>  ImageType;
  ImageType::Pointer image = ImageType::New();
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(20);

  ImageType::RegionType region1;
  region1.SetSize(size);
  region1.SetIndex(start);
  image->SetRegions(region1);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  // Make a square
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 10) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 10) )
        {
        imageIterator.Set(255);
        }
      else
        {
        imageIterator.Set(0);
        }

    ++imageIterator;
    }

  typedef itk::BinaryImageToLabelMapFilter<ImageType> BinaryImageToLabelMapFilterType;
  BinaryImageToLabelMapFilterType::Pointer binaryImageToLabelMapFilter = BinaryImageToLabelMapFilterType::New();
  binaryImageToLabelMapFilter->SetInput(image);
  binaryImageToLabelMapFilter->SetFullyConnected(true);
  binaryImageToLabelMapFilter->Update();

/////////////////TEST//////////////


std::cout << "BinaryWatershedTest Starting" << std::endl;

  return 0;
}
