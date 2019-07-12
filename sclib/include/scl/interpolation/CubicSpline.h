// -*- c++ -*-

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>
#include <math.h>

class CubicSpline
{
public:
	CubicSpline();
	CubicSpline(const std::vector<double> &Xsource, const std::vector<double> &Ysource, double FirstGradient = 0, double LastGradient = 0);
	~CubicSpline();

	void clear();  // ������
	bool isInit() { return m_isInit; }  // true : �X�v���C���v�Z�ς�, false : ���v�Z

	// �X�v���C���֐��̌v�Z
	void calc(const std::vector<double> &Xsource, const std::vector<double> &Ysource, double FirstGradient = 0, double LastGradient = 0);

	// �l�̎擾 (time = Xsource)
	double get(double time);

	// �T���v�����O���Ԃō��񂾃X�v���C���Ȑ��̑S�l���擾
	void sequence(std::vector<double> &seq, double sampling);
	std::vector<double> sequence(double sampling);

	// �R�s�[�I�y���[�^
	CubicSpline& operator=(const CubicSpline& si);

	// �����x�̃m�[�h
	const std::vector<double>& accList() const { return m_y2d; }


private:
	bool m_isInit;  // true : �X�v���C���v�Z�ς�, false : ���v�Z

	int m_nodeNumber;  // ��Ԃ���m�[�h�̐�
	std::vector<double> m_x, m_y;  // ��Ԃ���m�[�h(x : ����, y : �c��)
	std::vector<double> m_y2d;     // �X�v���C���֐�
};


#endif // ! CUBIC_SPLINE_H

