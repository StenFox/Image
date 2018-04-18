#include "CHistogram.h"
#include <cmath>

CHistogram::CHistogram( int _colPin )
{
    m_pin = _colPin;
    m_histogramms.resize(m_pin,0);
}

CHistogram::~CHistogram()
{

}

void CHistogram::addValueinPin( float _value, float _phi )
{
    float step = 2 * M_PI / m_pin;
    int pin = _phi / step;
    int pin2;
    if( _phi >= ( pin * step + step / 2 ) )
    {
        pin2 = pin + 1;
    }
    else
    {
        pin2 = pin - 1;
    }
    if( pin2 < 0 )
        pin2 = m_pin;
    if( pin2 > m_pin )
        pin2 = 0;

    if( pin > pin2 )
    {
        int temp = pin2;
        pin2 = pin;
        pin = temp;
    }

    float k1 = ( _phi - ( ( pin * step ) + step / 2 ) ) / step;
    float k2 = ( ( ( pin2 * step ) + step / 2 ) - _phi ) / step;

    k1 = 1 - k1;
    k2 = 1 - k2;

    m_histogramms[pin] += _value * k1;
    m_histogramms[pin2] += _value * k2;
}

void CHistogram::normalize( float _max )
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        m_histogramms[i] /= _max;
    }

    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        if( m_histogramms[i] > 0.2 )
            m_histogramms[i] = 0.2;
    }
}

void CHistogram::bound( float _val )
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        if( m_histogramms[i] > _val )
            m_histogramms[i] = _val;
    }
}

std::vector<float> CHistogram::getPeaks()
{
    float peak1 = -1;
    float peak2 = -1;
    size_t peak1index = -1;
    size_t peak2index = -1;

    std::vector<float> peaks;

    for( size_t i = 0; i < m_histogramms.size(); i++)
    {
        if( peak1 < m_histogramms[i] )
        {
            peak1 =  m_histogramms[i];
            peak1index = i;
        }
        if( peak2 < m_histogramms[i] && m_histogramms[i] < peak1 )
        {
            peak2 = m_histogramms[i];
            peak2index = i;
        }
    }
    peaks.push_back( pInterpolation( peak1index ) );
    if( peak2index != -1 && peak1 / peak2 >= 0.8 )
        peaks.push_back( pInterpolation( peak2index ) );
    return peaks;
}

float CHistogram::pInterpolation( const int _index )
{
    auto left = m_histogramms.at( ( _index - 1 + m_pin ) % m_pin );
    auto right = m_histogramms.at( ( _index + 1 ) % m_pin );
    auto mid = m_histogramms[ _index ];
    return ( left - right ) / ( 2 * ( left + right - 2 * mid ) );
}
