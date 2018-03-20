#ifndef CIMAGEHANDLER_H
#define CIMAGEHANDLER_H
#include "CImage.h"

enum mtProcessingEdgeEffects
{
    // Считаем что всё что "снаружи" = 0, т.е чёрным
    mtBlackEdge,

    // Копируем значения у границ
    mtCopyEdge,

    // Сворачиваем избражение самого в себя т.е создаём Тор
    mtThor,
};

class CImageHandler
{
public:
    CImageHandler();
private:

    // Размытие по Гауссу
    void gaussianBlur( float _sigma, mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Собеля
    void sobel( mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Привитта
    void priwitt( mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Робертса
    void robert( mtProcessingEdgeEffects _method = mtBlackEdge );

    // Билинейная интреполяция уменьшаем изображение в 2 раза
    void resizeTwo();
private:
    // Ядро Собель по X
    static CMatrixV<int> g_sobelX;

    // Ядро Собель по Y
    static CMatrixV<int> g_sobelY;

    // Ядро Привитт по X
    static CMatrixV<int> g_prewittX;

    // Ядро Привитт по Y
    static CMatrixV<int> g_prewittY;

    // Ядро Робертс по X
    static CMatrixV<int> g_robertX;

    // Ядро Робертс по Y
    static CMatrixV<int> g_robertY;

    // Функция перевода изображения в оттенки серого
    void grayScale( QImage _image );

    // Магнитуда или вычисление Величины градиента
    void magnitude( std::vector<int>& _input, const std::vector<int>& _gx, const std::vector<int>& _gy );

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    template<typename T>
    std::vector<int> convolution(const CMatrixV<T>& _kernel, mtProcessingEdgeEffects _method )
    {
        int kw = _kernel.getRows();
        int kh = _kernel.getColumns();
        auto offsetx = kw / 2;
        auto offsety = kw / 2;
        float sum;

        std::vector<int> outConvolution;
        outConvolution.reserve( m_height * m_width );

        for (auto y = 0; y < m_height; y++)
        {
            for (auto x = 0; x < m_width; x++)
            {
                sum = 0;
                for (auto j = 0; j < kh; j++)
                {
                    if ((y + j < offsety || y + j >= m_height))
                    {
                        if( _method == mtBlackEdge )
                            continue;
                    }
                    for (auto i = 0; i < kw; i++)
                    {
                        if ( (x + i < offsetx || x + i >= m_width) )
                        {
                            if( _method == mtBlackEdge )
                                continue;
                            if( _method == mtCopyEdge )
                            {
                                sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i ];
                                continue;
                            }
                            if( _method == mtThor )
                            {
                                if(y + j < offsety )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[m_height * m_width];
                                    }
                                    else if( x + i >= m_width )
                                    {
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ ( m_height - 1 ) * m_width ];
                                    }
                                    else
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                                    continue;
                                }

                                if( y + j >= m_height )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ m_width] ;
                                    }
                                    else if( x + i >= m_width )
                                    {
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ 0 ];
                                    }
                                    else
                                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                                    continue;
                                }
                            }
                        }
                        sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                    }
                }
                // Нормирование
                outConvolution.push_back( qBound(0x00, static_cast<int>(sum), 0xFF) );
            }
        }

        return outConvolution;
    }

    // Вычисление Гауссиана
    float gaussian( int _x,float _s );

    // Ядро фильтра гаусса
    std::vector<float> gaussianKernel( float _sigma );

    // Две последовательных свёртки для ускорния
    void convolutionForGauss( float _sigma, mtProcessingEdgeEffects _method );

    // Билинейная интерполяция
    std::vector<int> resizeBilinear( std::vector<int>& _image, int _widthOld, int heightOld, int _widthNew, int heightNew );
};

#endif // CIMAGEHANDLER_H
