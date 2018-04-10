#ifndef CHISTOGRAM_H
#define CHISTOGRAM_H
#include <vector>
#include <algorithm>
#include <QDebug>

class CHistogram
{
public:
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

private:
    int m_pin;  
    std::vector<float> m_histogramms;
};

#endif // CHISTOGRAM_H
