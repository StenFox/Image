#ifndef CMATRIXV_H
#define CMATRIXV_H
#include "vector"

template<typename T>
class CMatrixV
{
private:
    int m_columns;
    int m_rows;
    std::vector<T> m_matrix;
public:
    CMatrixV()
    {

    }



    CMatrixV( int _columns,int _rows )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix.resize( _columns * _rows, 0 );
    }

    CMatrixV& operator= ( const CMatrixV& _matrix )
    {

    }

    CMatrixV& operator= ( CMatrixV&& _matrix )
    {
        if (this != &_matrix)
        {
            m_matrix.clear();

            m_columns = _matrix.m_columns;
            m_rows = _matrix.m_rows;
            m_matrix = std::move( _matrix );

            _matrix.m_columns = 0;
            _matrix.m_rows = 0;
            _matrix.m_matrix.clear();
        }
        return *this;
    }

    CMatrixV( int _columns,int _rows,const std::vector<T>& _vectorForCopy )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix = _vectorForCopy;
    }

    int getColumns() const
    {
        return m_columns;
    }

    int getRows() const
    {
         return m_rows;
    }

    std::vector<T> getMatrix()
    {
        return m_matrix;
    }

    T getItem( int _currentColumns,int _currentRows ) const
    {
        return m_matrix[ _currentColumns * m_rows + _currentRows ];
    }

    void setItem( int _currentColumns,int _currentRows, T _value )
    {
        m_matrix[ _currentColumns * m_rows + _currentRows ] = _value;
    }
};

#endif // CMATRIXV_H
