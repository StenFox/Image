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
        m_matrix.reserve( _columns * _rows );
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
};

#endif // CMATRIXV_H
