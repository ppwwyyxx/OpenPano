#pragma once

#include <memory>
#include <iostream>

template <typename T>
class Mat {
    public:
				Mat(){}
        Mat(int rows, int cols, int channels);
        Mat(const Mat<T> &rhs);
        Mat<T> &operator=(const Mat<T> &rhs);

        T &at(int r, int c, int ch = 0);
        const T &at(int r, int c, int ch = 0) const;

        Mat<T> clone() const;

        const T *ptr(int r = 0) const
				{ return m_data.get() + r * m_cols * m_channels; }
        T *ptr(int r = 0)
				{ return m_data.get() + r * m_cols * m_channels; }
        const T *ptr(int r, int c) const
				{ return m_data.get() + (r * m_cols + c) * m_channels; }
        T *ptr(int r, int c)
				{ return m_data.get() + (r * m_cols + c) * m_channels; }
        int height() const { return m_rows; }
        int width() const { return m_cols; }
        int rows() const { return m_rows; }
        int cols() const { return m_cols; }
        int channels() const { return m_channels; }
				int pixels() const { return m_rows * m_cols; }

    private:
        int m_rows, m_cols;
        int m_channels;
        std::shared_ptr<T> m_data;
};

using Mat32f = Mat<float>;
extern template class Mat<float>;
