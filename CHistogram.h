#ifndef CHISTOGRAM_H
#define CHISTOGRAM_H
#include <vector>
#include <algorithm>
#include <QDebug>

class CHistogram
{
public:
    CHistogram();
    CHistogram( int _colPin );
    ~CHistogram();
    void addValueinPin( float _value, float _phi );
    int getColPin()
    {
        return m_pin;
    }

    float getPin(int _i)
    {
        return m_histogramms[_i];
    }

    void setPin( const int _colPin )
    {
        m_pin = _colPin;
        m_histogramms.resize( _colPin, 0 );
    }

    void normalize( float _max );

    void bound( float _val );

    float maxElement()
    {
        return *( std::max_element( m_histogramms.begin(), m_histogramms.end() ) );
    }

    float sumOfSquares()
    {
        float sum = 0;
        for( size_t i  = 0; i  < m_histogramms.size(); i++)
        {
            sum += m_histogramms[i] * m_histogramms[i];
        }
        return sum;
    }

    std::vector<float> getPeaks();

private:


    int m_pin;  
    std::vector<float> m_histogramms;

    float pInterpolation( const int _index );
};

#endif // CHISTOGRAM_H
