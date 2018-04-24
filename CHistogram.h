#ifndef CHISTOGRAM_H
#define CHISTOGRAM_H
#include <algorithm>

class CHistogram
{
public:
    CHistogram() = default;
    CHistogram( int _colBasket );
    CHistogram( CHistogram&& _his) = default;
    CHistogram( const CHistogram& _his) = default;
    CHistogram &operator=( CHistogram&& _his ) = default;
    CHistogram &operator=( const CHistogram& _his ) = default;
    ~CHistogram() = default;

    void addValueinBasket( float _value, float _phi );

    int getColBasket();

    float getValueinBasket(const int _i);

    void setColBasket( const int _colBasket );

    void normalize( float _max );

    void bound( float _val );

    float maxElement();

    float sumOfSquares();

    std::vector<float> getPeaks();

    void clear();

private:
    std::vector<float> m_histogramms;
    float basketIterpolation( const int _index );
};

#endif // CHISTOGRAM_H
