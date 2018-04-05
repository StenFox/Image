#ifndef CPYRAMID_H
#define CPYRAMID_H
#include "CImage.h"
#include <vector>

class CPyramid
{
public:
    CPyramid( int _octaves );
    void setImageInOctaves( CImage& _myImage, int octava )
    {
        m_pyramidImage[octava] = _myImage;
    }

    CImage getImageInOctaves( int octava )
    {
        return m_pyramidImage[octava];
    }

    int getOctaves()
    {
        return m_octaves;
    }

private:
    int m_octaves;
    std::vector<CImage> m_pyramidImage;
};

#endif // CPYRAMID_H
