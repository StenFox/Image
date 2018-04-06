#include "CPyramid.h"

CPyramid::CPyramid( int _octaves, float _sigmaZero, int _scales )
{
    m_octaves = _octaves;
    m_pyramidImage.resize( m_octaves, CImage() );
    m_sigmas.resize( m_octaves, 0 );
    m_evectifSigma = _sigmaZero;
    m_sclaes = _scales;
}
