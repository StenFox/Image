#ifndef CMATRIXV_H
#define CMATRIXV_H
#include <vector>
#include <algorithm>

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
        m_columns = 0;
        m_rows = 0;
        m_matrix.resize( 0, 0 );
    }

    CMatrixV( int _columns,int _rows )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix.resize( _columns * _rows, 0 );
    }

    CMatrixV& operator= ( const CMatrixV& _matrix )
    {
        m_columns = _matrix.m_columns;
        m_rows = _matrix.m_rows;
        m_matrix.resize( m_rows * m_columns );
        std::copy( _matrix.m_matrix.begin(), _matrix.m_matrix.end(), m_matrix.begin() );
        return *this;
    }

    CMatrixV& operator= ( CMatrixV&& _matrix )
    {
        if (this != &_matrix)
        {
            m_matrix.clear();
            m_columns = 0;
            m_rows = 0;

            m_columns = std::move( _matrix.m_columns );
            m_rows = std::move( _matrix.m_rows );
            m_matrix = std::move( _matrix.m_matrix );

        }
        return *this;
    }

    CMatrixV( int _columns,int _rows,const std::vector<T>& _vectorForCopy )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix = std::move( _vectorForCopy );
    }

    ~CMatrixV()
    {
        m_columns = 0;
        m_rows = 0;
        m_matrix.clear();
    }

    int getColumns() const
    {
        return m_columns;
    }

    void setColumns( int _columns )
    {
        m_columns = _columns;
    }

    int getRows() const
    {
         return m_rows;
    }

    void setRows( int _rows )
    {
        m_rows = _rows;
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

    void normalize()
    {
        auto it_min = std::min_element( m_matrix.begin(), m_matrix.end() );
        T min = *it_min;
        if( min<0 )
            for( size_t i = 0; i < m_matrix.size(); i++)
            {
                m_matrix[i] += min;
            }

        auto it_max = std::max_element( m_matrix.begin(),m_matrix.end() );
        T max = *it_max;
        if( max > 255 )
        for( size_t i = 0; i < m_matrix.size(); i++)
        {
            m_matrix[i] /= max;
            m_matrix[i] *= 255;
        }
    }

    void oneNormalize()
    {
        auto it_min = std::min_element( m_matrix.begin(),m_matrix.end() );
        T min = *it_min;
        if( min < 0 )
            for (size_t i = 0; i < m_matrix.size(); i++)
            {
                m_matrix[i] += min;
            }

        auto it_max = std::max_element( m_matrix.begin(),m_matrix.end() );
        T max = *it_max;
        for( size_t i = 0; i < m_matrix.size(); i++ )
        {
            m_matrix[i] /= max;
        }
    }

    void resize( int _columns,int _rows,const std::vector<T>& _vectorForCopy )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix = std::move( _vectorForCopy );
    }

    void resize( int _columns,int _rows )
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix.resize(m_columns*m_rows);
    }

    bool isValid( int _columns,int _rows ) const
    {
        if( _columns < 0 || _rows < 0)
            return false;
        if( _columns > m_columns - 1 )
            return false;
        if( _rows > m_rows - 1 )
            return false;
        return true;
    }
};

#endif // CMATRIXV_H
