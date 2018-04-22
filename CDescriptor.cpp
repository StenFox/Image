#include "CDescriptor.h"

CDescriptor::CDescriptor( int _colPin, int _colHistogramm  )
{
    m_descriptor.resize( _colHistogramm, CHistogram( _colPin ) );
}

CDescriptor::CDescriptor( const QPoint& _interestPoint, int _colPin, int _colHistogramm  )
{
    m_intrestPoint = _interestPoint;
    m_descriptor.resize( _colHistogramm, CHistogram( _colPin ) );
}

void CDescriptor::addValueInHistogramm( float _value, float _phi, int _histogramm )
{
    m_descriptor[_histogramm].addValueinBasket( _value, _phi );
}

void CDescriptor::setInterestPoint( const QPoint& _interestPoint )
{
     m_intrestPoint = _interestPoint;
}

void CDescriptor::setColHistogramm( int _colPin, int _colHistogramm )
{
     m_descriptor.clear();
     m_descriptor.resize( _colHistogramm, CHistogram( _colPin ) );
}

void CDescriptor::normalize( float _value )
{
    float l2 = 0;
    for( size_t i = 0; i < m_descriptor.size(); i++ )
    {
        l2 += m_descriptor[i].sumOfSquares();
    }
    l2 = sqrt( l2 );
    for( size_t i = 0; i < m_descriptor.size(); i++ )
    {
        m_descriptor[i].normalize( l2 );
        m_descriptor[i].bound( _value );
    }

    l2 = 0;
    for( size_t i = 0; i < m_descriptor.size(); i++ )
    {
        l2 += m_descriptor[i].sumOfSquares();
    }
    l2 = sqrt( l2 );
    for( size_t i = 0; i < m_descriptor.size(); i++ )
    {
        m_descriptor[i].normalize( l2 );
    }
}
