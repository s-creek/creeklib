#ifndef B_SPLINE_H
#define B_SPLINE_H

#include <vector>

class BSpline
{
public:
	BSpline(unsigned int dim = 3) { m_dim = dim;  m_searchType = false; }

	void setDim(unsigned int dim) { m_dim = dim; }

	// �X�v���C���֐��̌v�Z (���ۂɂ�get�̕������C���E�E�E)
	//   Xsource : ���� (���ԂƂ�)
	//   Ysource : �c�� (�ʒu�Ƃ�)
	bool calc(const std::vector<double> &Xsource, const std::vector<double> &Ysource, const int knotType = 0);

	// �l�̎擾 (����B�X�v���C���̌v�Z�����Ă���)
	//   x = time
	//   y = f(x)
	double get(double time);

	// ������
	void clear();

	// �R�s�[�I�y���[�^
	BSpline& operator=(const BSpline& si);


private:
	void openKnot();     // �J�m�b�g�x�N�g�����v�Z (���Ԃ�}��ϐ��ɂ��邽�߂Ɉ�l�ɂ͂Ȃ��ĂȂ�)
	void uniformKnot();  // ��l�m�b�g�x�N�g�����v�Z

	// B-�X�v���C�����֐��iB-spline basis function�j
	// �ϐ����� wikipedia �ɍ��킹�� ( b_i,i(t) )
	double bsplineBasisFunction(const unsigned int j, const unsigned int k, const double t, const std::vector<double>& knot);


	unsigned int m_dim;  // ��Ԏ��̎���
	int m_knotType;      // �g���m�b�g�x�N�g���̃^�C�v (0 : openKnot, 1 : uniformKnot)
	std::vector<double> m_knot;    // �m�b�g�x�N�g��
	std::vector<double> m_x, m_y;  // ��Ԃ���m�[�h(x : ����, y : �c��)

	bool m_searchType;  // ���֐��̌v�Z�͈� ( true : �񕪊�T��, false : �P���O���T�� )
};

#endif // !B_SPLINE_H
