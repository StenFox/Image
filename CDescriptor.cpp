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
