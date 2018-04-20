#ifndef CDESCRIPTOR_H
#define CDESCRIPTOR_H
#include "CHistogram.h"
#include <QPoint>
#include <vector>

class CDescriptor
{
public:
    CDescriptor( int _colPin, int _colHistogramm );
    CDescriptor( const QPoint& _interestPoint, int _colPin, int _colHistogramm );
    CDescriptor( CDescriptor&& _des) = default;
    CDescriptor( const CDescriptor& _des) = default;
    CDescriptor &operator=(CDescriptor&& _des) = default;
    CDescriptor &operator=(const CDescriptor& _des) = default;

    void addValueInHistogramm( float _value, float _phi, int _histogramm );
    void setInterestPoint( const QPoint& _interestPoint );
    void setColHistogramm( int _colPin, int _colHistogramm );

    int getColHistogramms() const
    {
        return m_descriptor.size();
    }

    CHistogram getHistograms(int _i) const
    {
        return m_descriptor[_i];
    }

    QPoint getInterestPoint() const
    {
        return m_intrestPoint;
    }

    void normalize( float _value );
private:
    QPoint m_intrestPoint;
    int m_pin;
    std::vector<CHistogram> m_descriptor;
};

#endif // CDESCRIPTOR_H
