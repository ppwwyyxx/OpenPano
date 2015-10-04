#pragma once

#include <memory>
#include <iostream>
#include <cstring>
#include "lib/debugutils.hh"

template <typename T>
class Mat {
    public:
				Mat(){}
				Mat(int rows, int cols, int channels):
					m_rows(rows), m_cols(cols), m_channels(channels),
					m_data(new T[rows * cols * channels], [](T *d) { delete[] d; })
				{
					//memset(m_data.get(), 0, sizeof(T) * rows * cols * channels);
				}

				Mat(const Mat<T> &rhs):
					m_rows(rhs.m_rows), m_cols(rhs.m_cols), m_channels(rhs.m_channels),
					m_data(rhs.m_data) { }

				Mat<T> &operator=(const Mat<T> &rhs) {
					this->m_rows = rhs.m_rows;
					this->m_cols = rhs.m_cols;
					this->m_channels = rhs.m_channels;
					this->m_data = rhs.m_data;
					return *this;
				}

				virtual ~Mat(){}

				T &at(int r, int c, int ch = 0) {
					m_assert(r < m_rows);
					m_assert(c < m_cols);
					m_assert(ch < m_channels);
					return ptr(r)[c * m_channels + ch];
				}

				const T &at(int r, int c, int ch = 0) const {
					m_assert(r < m_rows);
					m_assert(c < m_cols);
					m_assert(ch < m_channels);
					return ptr(r)[c * m_channels + ch];
				}

				Mat<T> clone() const {
					Mat<T> res(m_rows, m_cols, m_channels);
					memcpy(res.ptr(0), this->ptr(0), sizeof(T) * m_rows * m_cols * m_channels);
					return res;
				}

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
