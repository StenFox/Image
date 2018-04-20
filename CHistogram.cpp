#include "CHistogram.h"
#include <cmath>

CHistogram::CHistogram( int _colPin )
{
    m_pin = _colPin;
    m_histogramms.resize( m_pin );
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

int CHistogram::getColPin()
{
    return m_pin;
}

float CHistogram::getValueinPin( const int _i )
{
    return m_histogramms[_i];
}

void CHistogram::setColPin( const int _colPin )
{
    m_pin = _colPin;
    m_histogramms.resize( _colPin, 0 );
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

void CHistogram::bound( const float _val )
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        if( m_histogramms[i] > _val )
            m_histogramms[i] = _val;
    }
}    

float CHistogram::maxElement()
{
    return *( std::max_element( m_histogramms.begin(), m_histogramms.end() ) );
}

float CHistogram::sumOfSquares()
{
    float sum = 0;
    for( size_t i  = 0; i < m_histogramms.size(); i++ )
    {
        sum += m_histogramms[i] * m_histogramms[i];
    }
    return sum;
}

std::vector<float> CHistogram::getPeaks()
{
    float peak1 = -1;
    float peak2 = -1;
    size_t peak1index = -1;
    size_t peak2index = -1;

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
    std::vector<float> peaks;
    peaks.push_back( pInterpolation( peak1index ) );
    if( peak2index != -1 && peak1 / peak2 >= 0.8 )
        peaks.push_back( pInterpolation( peak2index ) );
    return peaks;
}

void CHistogram::clear()
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        m_histogramms[i] = 0;
    }
}

float CHistogram::pInterpolation( const int _index )
{
    auto left = m_histogramms.at( ( _index - 1 + m_pin ) % m_pin );
    auto right = m_histogramms.at( ( _index + 1 ) % m_pin );
    auto mid = m_histogramms[ _index ];
    return ( left - right ) / ( 2 * ( left + right - 2 * mid ) );
}
