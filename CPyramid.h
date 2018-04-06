#ifndef CPYRAMID_H
#define CPYRAMID_H
#include "CImage.h"
#include <vector>

class CPyramid
{
public:

    CPyramid( int _octaves, float _sigmaZero, int _scales );

    void setImageInOctaves( CImage& _myImage, int _octava, float _sigma )
    {
        m_pyramidImage[_octava] = _myImage;
        m_sigmas[_octava] = _sigma;
        m_evectifSigma *= _sigma;
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
    int m_sclaes;
    float m_evectifSigma;
    std::vector<float> m_sigmas;
    std::vector<CImage> m_pyramidImage;
};

#endif // CPYRAMID_H
