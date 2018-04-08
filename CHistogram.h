#ifndef CHISTOGRAM_H
#define CHISTOGRAM_H
#include <vector>

class CHistogram
{
public:
    CHistogram( int _colPin );
    ~CHistogram();
    void addValueinPin( float _value, float _phi );
private:
    int m_pin;
    std::vector<float> m_histogramms;
};

#endif // CHISTOGRAM_H
