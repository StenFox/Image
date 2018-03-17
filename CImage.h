#ifndef CIMAGE_H
#define CIMAGE_H
#include <vector>
#include <QImage>
#include "QPixmap"
#include "CMatrixV.h"

enum mtProcessingEdgeEffects
{
    // Считаем что всё что "снаружи" = 0, т.е чёрным
    mtBlackEdge,

    // Копируем значения у границ
    mtCopyEdge,

    // Сворачиваем избражение самого в себя т.е создаём Тор
    mtThor,
};

class CImage
{
public:
    CImage(QString _fileName);
    QImage getImage();
    QImage getOriginalImage();

    // Размытие по Гауссу
    void GaussianBlur(float _sigma);

    // Оператор Собеля
    void Sobel();

    // Билинейная интреполяция уменьшаем изображение в 2 раза
    void resizeTwo();
private:

    // Ядро Собель по X
    static CMatrixV<int> g_sobelX;

    // Ядро Собель по Y
    static CMatrixV<int> g_sobelY;

    // Ширина "моего изображения"
    int m_width;

    // Высота "моего изображения"
    int m_height;

    //Оригинальное изображение
    QImage m_originalImage;

    //Вектор для хранения изображения
    std::vector<int> m_myImage;

    // Функция перевода изображения в оттенки серого
    void GrayScale(QImage _image);

    // Магнитуда или вычисление Величины градиента
    void magnitude(std::vector<int>& input, const std::vector<int>& gx, const std::vector<int>& gy);

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    std::vector<int> convolution(CMatrixV<auto> kernel, mtProcessingEdgeEffects _method = mtBlackEdge );

    float gaussian(int _x,float _s);

    std::vector<float> GaussianKernel(float _sigma);

    void ConvolutionForGauss(float _sigma);

    std::vector<int> resizeBilinear(std::vector<int> pixels, int w, int h, int w2, int h2);
};

#endif // CIMAGE_H
