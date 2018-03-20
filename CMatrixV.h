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
    CMatrixV(int _columns,int _rows)
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix.resize( _columns * _rows, 0 );
    }

    CMatrixV(int _columns,int _rows,std::vector<T> _vectorForCopy)
    {
        m_columns = _columns;
        m_rows = _rows;
        m_matrix = _vectorForCopy;
    }

    int getColumns()
    {
        return m_columns;
    }

    int getRows()
    {
         return m_rows;
    }

    std::vector<T> getMatrix()
    {
        return m_matrix;
    }

    T getItem(int _currentColumns,int _currentRows)
    {
        return m_matrix[ _currentColumns * m_rows + _currentRows ];
    }
};

#endif // CMATRIXV_H
