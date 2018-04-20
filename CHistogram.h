#ifndef CHISTOGRAM_H
#define CHISTOGRAM_H
#include <vector>
#include <algorithm>

class CHistogram
{
public:
    CHistogram( int _colPin );
    CHistogram( CHistogram&& _his) = default;
    CHistogram( const CHistogram& _his) = default;
    CHistogram &operator=(CHistogram&& _his) = default;
    CHistogram &operator=(const CHistogram& _his) = default;

    void addValueinPin( float _value, float _phi );

    int getColPin();

    float getValueinPin(const int _i);

    void setColPin( const int _colPin );

    void normalize( float _max );

    void bound( float _val );

    float maxElement();

    float sumOfSquares();

    std::vector<float> getPeaks();

    void clear();

private:
    int m_pin;
    std::vector<float> m_histogramms;
    float pInterpolation( const int _index );
};

#endif // CHISTOGRAM_H
