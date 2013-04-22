// File: stitcher.hh
// Date: Mon Apr 22 19:02:01 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <memory>
#include "image.hh"
#include "matrix.hh"

class Stitcher {
	private:
		const std::shared_ptr<Img> & img1, & img2;
		const Matrix & trans;
	public:
		// trans: img2 -> img1
		Stitcher(const std::shared_ptr<Img>& m_img1, const std::shared_ptr<Img>& m_img2, const Matrix &m_trans):
			img1(m_img1), img2(m_img2), trans(m_trans){ }

		std::shared_ptr<Img> stitch() const;

};
