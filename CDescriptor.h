#ifndef CDESCRIPTOR_H
#define CDESCRIPTOR_H
#include "CHistogram.h"
#include <QPoint>
#include <vector>

class CDescriptor
{
public:
    CDescriptor();
    ~CDescriptor();
    CDescriptor( int _colPin, int _colHistogramm );
    CDescriptor( QPoint _interestPoint ,int _colPin, int _colHistogramm );
    void addValueInHistogramm( float _value, float _phi, int _histogramm );
    void setInterestPoint( QPoint _interestPoint );
private:
    QPoint m_intrestPoint;
    int m_pin;
    std::vector<CHistogram> m_descriptor;
};

#endif // CDESCRIPTOR_H
