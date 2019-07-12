#include "BSpline.h"
#include <iostream>

bool BSpline::calc(const std::vector<double> &Xsource, const std::vector<double> &Ysource, const int knotType)
{
	// �m�[�h�̐�������Ȃ�
	if (Ysource.size() <= m_dim) {
		return false;
	}

	// x �� y �̃m�[�h������v���Ȃ�
	if (Xsource.size() != Ysource.size()) {
		return false;
	}

	m_x.clear();
	m_y.clear();

	unsigned int nodeNumber = Ysource.size();

	// �m�[�h���������Ƃ��͓񕪊�T��
	if (nodeNumber > 3 * m_dim) {
		m_searchType = true;
	}
	// �m�[�h�������Ȃ��Ƃ��͒P���O���T��
	else {
		m_searchType = false;
	}


	//�ϐ��̈���m��
	m_x.resize(nodeNumber);
	m_y.resize(nodeNumber);

	m_x = Xsource;
	m_y = Ysource;


	// �m�b�g�x�N�g���̌v�Z
	if (knotType == 1) {
		m_knotType = 1;
		uniformKnot();
	}
	else {
		m_knotType = 0;
		openKnot();
	}
	return true;
}


double BSpline::get(double time)
{
	// �n�_�ȑO�̃f�[�^�͑S���n�_�Ƃ��Čv�Z
	const static double NEAR_ZERO(1.0e-6);
	if (time - NEAR_ZERO <= m_x.front()) {
		time = m_x.front();

		// �J�m�b�g�x�N�g�����Ǝn�_�Ȃ̂ł��̂܂ܕԂ�
		if (m_knotType == 0) {
			return m_y.front();
		}
	}
	// �I�_�ȍ~�̃f�[�^�͑S���I�_�Ƃ��Čv�Z
	else if (time + NEAR_ZERO >= m_x.back()) {
		time = m_x.back();

		// �J�m�b�g�x�N�g�����ƏI�_�Ȃ̂ł��̂܂ܕԂ�
		if (m_knotType == 0) {
			return m_y.back();
		}
	}


	// �Ԃ�l
	double s(0.0);
	if (m_searchType) {
		// ���֐����[���ȊO�͈̔͒T��
		// �����̃��[�v�Ƃǂ������������E�E�E�H
		unsigned int klo, khi, k;
		klo = m_dim;
		khi = m_knot.size() - m_dim - 1;

		// 2���T���A���S���Y�����[�v�@�v�Z�ʁF����log2N��̎��s
		while (khi - klo > 1) {
			k = (khi + klo) >> 1;
			if (m_knot[k] > time) khi = k;
			else klo = k;
		}

		// �Ԃ�l�v�Z
		for (unsigned int i = 0; i < m_dim + 1; i++) {
			s += bsplineBasisFunction(i + klo - m_dim, m_dim, time, m_knot) * m_y.at(i + klo - m_dim);
		}
	}
	else {
		for (unsigned int i = 0; i < m_y.size(); i++) {
			s += bsplineBasisFunction(i, m_dim, time, m_knot) * m_y.at(i);
		}
	}
	return s;
}


void BSpline::clear()
{
	m_knot.clear();
	m_x.clear();
	m_y.clear();
}


void BSpline::openKnot()
{
	m_knot.clear();
	m_knot.resize(m_x.size() + m_dim + 1);

	unsigned int i = 0;
	for (; i < m_dim + 1; i++) {
		m_knot.at(i) = m_x.front();
	}
	for (; i < m_knot.size() - m_dim - 1; i++) {
		double t(0);
		unsigned int k(i-m_dim);
		for (unsigned int j = 0; j < m_dim; j++) {
			t += m_x.at(k);
			k++;
		}
		t /= (double)m_dim;
		m_knot.at(i) = t;
	}
	for (; i < m_knot.size(); i++) {
		m_knot.at(i) = m_x.back();
	}


	return;


	//
	// debug
	//
	for (unsigned int i = 0; i < m_knot.size(); i++) {
		std::cout << "  " << m_knot.at(i) << std::endl;
	}
	std::cout << "---------------------------------------" << std::endl;
	for (unsigned int i = 1; i < m_knot.size(); i++) {
		double t = 0.5 * (m_knot.at(i) + m_knot.at(i-1));
		std::cout << t << " : ";
		for (unsigned int j = 0; j < m_y.size(); j++) {
			std::cout << "  " << bsplineBasisFunction(j, m_dim, t, m_knot);
		}
		std::cout << std::endl;
	}
	std::cout << "---------------------------------------" << std::endl;
	const static double NEAR_ZERO(1.0e-6);
	for (unsigned int i = 0; i < m_knot.size(); i++) {
		double thi, tlo;
		thi = m_knot.at(i) + NEAR_ZERO;
		tlo = m_knot.at(i) - NEAR_ZERO;

		std::cout << tlo << " : ";
		for (unsigned int j = 0; j < m_y.size(); j++) {
			std::cout << "  " << bsplineBasisFunction(j, m_dim, tlo, m_knot);
		}
		std::cout << std::endl;

		std::cout << thi << " : ";
		for (unsigned int j = 0; j < m_y.size(); j++) {
			std::cout << "  " << bsplineBasisFunction(j, m_dim, thi, m_knot);
		}
		std::cout << std::endl;
		std::cout << "---------------------------------------" << std::endl;
	}
}


void BSpline::uniformKnot()
{
	m_knot.clear();

	unsigned int m(m_x.size() + m_dim + 1);
	m_knot.resize(m);

	double t0, gradient;
	gradient = (m_x.back() - m_x.front()) / (m_x.size() - m_dim);
	t0 = m_x.front() - gradient * m_dim;

	for (unsigned int i = 0; i < m; i++) {
		m_knot.at(i) = t0 + gradient * i;
	}
	m_knot.at(m_dim) = m_x.front();
	m_knot.at(m - m_dim - 1) = m_x.back();


	return;


	//
	// debug
	//
	for (unsigned int i = 0; i < m_knot.size(); i++) {
		std::cout << "  " << m_knot.at(i) << std::endl;
	}
	std::cout << "---------------------------------------" << std::endl;
}


// B-�X�v���C�����֐��iB-spline basis function�j
double BSpline::bsplineBasisFunction(const unsigned int j, const unsigned int k, const double t, const std::vector<double>& knot)
{
	if (k == 0) {
		if (knot.at(j) <= t && t < knot.at(j + 1)) {
			return 1;
		}
		else {
			return 0;
		}
	}

	double w1(0), w2(0);
	if (knot.at(j + k) - knot.at(j) > 1.0e-8) {
		w1 = (t - knot.at(j)) / (knot.at(j + k) - knot.at(j)) * bsplineBasisFunction(j, k - 1, t, knot);
	}
	if (knot.at(j + k + 1) - knot.at(j + 1) > 1.0e-8) {
		w2 = (knot.at(j + k + 1) - t) / (knot.at(j + k + 1) - knot.at(j + 1)) * bsplineBasisFunction(j + 1, k - 1, t, knot);
	}

	return w1 + w2;
}