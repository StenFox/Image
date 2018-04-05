#include "CPyramid.h"

CPyramid::CPyramid( int _octaves )
{
    m_octaves = _octaves;
    m_pyramidImage.resize( m_octaves, CImage() );
}
