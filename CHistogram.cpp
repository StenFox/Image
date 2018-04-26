#include "CHistogram.h"
#include <cmath>

CHistogram::CHistogram( int _colBasket )
{
    m_histogramms.resize( _colBasket, 0 );
}

void CHistogram::addValueinBasket( float _value, float _phi )
{
    int basket = 0;
    int basket2 = 0;
    float step = 2 * M_PI / m_histogramms.size();
    basket = _phi / step;
    if( basket == m_histogramms.size() )
        basket = m_histogramms.size() -1;

    if( _phi >= ( basket * step + step / 2 ) )
    {
        basket2 = basket + 1;
    }
    else
    {
        basket2 = basket - 1;
    }

    if( basket2 < 0 )
        basket2 = 1;
    if( basket2 > m_histogramms.size() - 1 )
        basket2 = m_histogramms.size() - 2;

    if( basket > basket2 )
    {
        int temp = basket2;
        basket2 = basket;
        basket = temp;
    }

    float k1 = ( _phi - ( ( basket * step ) + step / 2 ) ) / step;
    k1 = 1 - k1;
    float k2 = 1 - k1;
    try
    {
        m_histogramms.at( basket ) += _value * k1;
        m_histogramms.at( basket2 ) += _value * k2;
    }
    catch(std::exception ex)
    {
    }
}

int CHistogram::getColBasket()
{
    return m_histogramms.size();
}

float CHistogram::getValueinBasket( const int _i )
{
    return m_histogramms.at( _i );
}

void CHistogram::setColBasket( const int _colBasket )
{
    m_histogramms.clear();
    m_histogramms.resize( _colBasket, 0 );
}

void CHistogram::normalize( float _max )
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        m_histogramms.at(i) /= _max;
        m_histogramms.at(i) /= _max;
    }
}

void CHistogram::bound( const float _val )
{
    for( size_t i = 0; i < m_histogramms.size(); i++ )
    {
        if( m_histogramms.at(i) > _val )
            m_histogramms.at(i) = _val;
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
        sum += m_histogramms.at(i) * m_histogramms.at( i );
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
        if( peak1 < m_histogramms.at(i) )
        {
            peak1 =  m_histogramms.at(i);
            peak1index = i;
        }
        if( peak2 < m_histogramms.at(i) && m_histogramms.at(i) < peak1 )
        {
            peak2 = m_histogramms.at(i);
            peak2index = i;
        }
    }
    std::vector<float> peaks;
    peaks.push_back( basketIterpolation( peak1index ) );
    if( peak2index != -1 && peak1 / peak2 >= 0.8 )
        peaks.push_back( basketIterpolation( peak2index ) );
    return peaks;
}

void CHistogram::clear()
{
    m_histogramms.clear();
}

float CHistogram::basketIterpolation( const int _index )
{
    auto size = m_histogramms.size();
    auto left = m_histogramms.at( ( _index - 1 + size ) % size );
    auto right = m_histogramms.at( ( _index + 1 ) % size );
    auto mid = m_histogramms.at( _index );
    return ( left - right ) / ( 2 * ( left + right - 2 * mid ) );
}
