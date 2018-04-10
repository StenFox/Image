#include "CDescriptor.h"

CDescriptor::CDescriptor()
{
}

CDescriptor::~CDescriptor()
{
    m_descriptor.clear();
}

CDescriptor::CDescriptor( int _colPin, int _colHistogramm  )
{
    m_pin = _colPin;
    m_descriptor.resize( _colHistogramm, CHistogram( m_pin ) );
}

CDescriptor::CDescriptor( QPoint _interestPoint, int _colPin, int _colHistogramm  )
{
    m_intrestPoint = _interestPoint;
    m_pin = _colPin;
    m_descriptor.resize( _colHistogramm,CHistogram( _colPin ));
}

void CDescriptor::addValueInHistogramm( float _value, float _phi, int _histogramm)
{
    m_descriptor[_histogramm].addValueinPin( _value, _phi );
}

void CDescriptor::setInterestPoint( QPoint _interestPoint )
{
     m_intrestPoint = _interestPoint;
}

void CDescriptor::normalize()
{
    float l2 = 0;
    for( int i = 0; i < m_descriptor.size(); i++ )
    {
        l2 += m_descriptor[i].sumOfSquares();
    }
    l2 = sqrt(l2);
    for( int i = 0; i < m_descriptor.size(); i++ )
    {
        m_descriptor[i].normalize( l2 );
        m_descriptor[i].bound(0.2);
    }

    l2 = 0;
    for( int i = 0; i < m_descriptor.size(); i++ )
    {
        l2 += m_descriptor[i].sumOfSquares();
    }
    l2 = sqrt( l2 );
    for( int i = 0; i < m_descriptor.size(); i++ )
    {
        m_descriptor[i].normalize( l2 );
    }
}
