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
				virtual ~Mat(){}

        T &at(int r, int c, int ch = 0);
        const T &at(int r, int c, int ch = 0) const;

        Mat<T> clone() const;

        inline const T *ptr(int r = 0) const
				{ return m_data.get() + r * m_cols * m_channels; }
        inline T *ptr(int r = 0)
				{ return m_data.get() + r * m_cols * m_channels; }
        inline const T *ptr(int r, int c) const
				{ return m_data.get() + (r * m_cols + c) * m_channels; }
        inline T *ptr(int r, int c)
				{ return m_data.get() + (r * m_cols + c) * m_channels; }
        inline int height() const { return m_rows; }
        inline int width() const { return m_cols; }
        inline int rows() const { return m_rows; }
        inline int cols() const { return m_cols; }
        inline int channels() const { return m_channels; }
				inline int pixels() const { return m_rows * m_cols; }

    protected:
        int m_rows, m_cols;
        int m_channels;
        std::shared_ptr<T> m_data;
};

using Mat32f = Mat<float>;
