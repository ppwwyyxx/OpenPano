#include "mat.h"
#include "debugutils.hh"

#include <cstring>
#include <iomanip>

template <typename T>
Mat<T>::Mat(int rows, int cols, int channels):
    m_rows(rows), m_cols(cols), m_channels(channels),
    m_data(new T[rows * cols * channels], [](T *d) { delete[] d; })
{
    m_assert(channels == 1 || channels == 3);
    //memset(m_data.get(), 0, sizeof(T) * rows * cols * channels);
}

template <typename T>
Mat<T>::Mat(const Mat<T> &rhs):
    m_rows(rhs.m_rows), m_cols(rhs.m_cols), m_channels(rhs.m_channels),
    m_data(rhs.m_data) { }

template <typename T>
Mat<T> &Mat<T>::operator=(const Mat<T> &rhs) {
    this->m_rows = rhs.m_rows;
    this->m_cols = rhs.m_cols;
    this->m_channels = rhs.m_channels;
    this->m_data = rhs.m_data;
    return *this;
}

template <typename T>
T &Mat<T>::at(int r, int c, int ch)
{
    m_assert(r < m_rows);
    m_assert(c < m_cols);
    m_assert(ch < m_channels);
    return ptr(r)[c * m_channels + ch];
}

template <typename T>
const T &Mat<T>::at(int r, int c, int ch) const {
    m_assert(r < m_rows);
    m_assert(c < m_cols);
    m_assert(ch < m_channels);
    return ptr(r)[c * m_channels + ch];
}

template <typename T>
Mat<T> Mat<T>::clone() const {
    Mat<T> res(m_rows, m_cols, m_channels);
		memcpy(res.ptr(0), this->ptr(0), sizeof(T) * m_rows * m_cols * m_channels);
    return res;
}

template class Mat<float>;
template class Mat<double>;
