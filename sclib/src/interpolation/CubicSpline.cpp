#include "CubicSpline.h"
//#include <iostream>

using namespace std;

#define NEAR_ZERO 1.0e-6

void CubicSpline::clear()
{
	m_isInit = false;
	m_nodeNumber = 0;
	m_x.clear();
	m_y.clear();
	m_y2d.clear();
}


CubicSpline::~CubicSpline()
{
	clear();
}


CubicSpline::CubicSpline()
{
	clear();
}


CubicSpline::CubicSpline(const vector<double> &Xsource, const vector<double> &Ysource, double FirstGradient, double LastGradient)
{
	clear();
	calc(Xsource, Ysource, FirstGradient, LastGradient);
}


void CubicSpline::calc(const vector<double> &Xsource, const vector<double> &Ysource, double FirstGradient, double LastGradient)
{
	int i, j;
	double p, qn, sig, un;
	vector<double> u;

	m_isInit = false;

	// 例外処理
	if (Xsource.size() != Ysource.size()) {
		//cout << "Size is different !!" << " x : " << Xsource.size() << " y : " << Ysource.size() << endl;
		return;
	}
	else if (Xsource.size() < 2) {
		//cout << "Size is not enough !!" << " size : " << Xsource.size() << " < 2" << endl;
		return;
	}

	clear();

	m_nodeNumber = Xsource.size();

	//変数領域を確保
	m_x.resize(m_nodeNumber);
	m_y.resize(m_nodeNumber);
	m_y2d.resize(m_nodeNumber);
	u.resize(m_nodeNumber);

	m_x = Xsource;
	m_y = Ysource;

	// 1.00e30以上の始端の傾きが入力された場合は，始端は自然スプラインとする
	if (FirstGradient > 0.99e30)
		m_y2d[0] = u[0] = 0.0;
	else {
		m_y2d[0] = -0.5;
		u[0] = (3.0 / (m_x[1] - m_x[0]))*((m_y[1] - m_y[0]) / (m_x[1] - m_x[0]) - FirstGradient);
	}
	// 3重対角アルゴリズム分解ループ　計算量：O(N)
	for (i = 1; i<m_nodeNumber - 1; i++) {
		sig = (m_x[i] - m_x[i - 1]) / (m_x[i + 1] - m_x[i - 1]);
		p = sig*m_y2d[i - 1] + 2.0;
		m_y2d[i] = (sig - 1.0) / p;
		u[i] = (m_y[i + 1] - m_y[i]) / (m_x[i + 1] - m_x[i]) - (m_y[i] - m_y[i - 1]) / (m_x[i] - m_x[i - 1]);
		u[i] = (6.0*u[i] / (m_x[i + 1] - m_x[i - 1]) - sig*u[i - 1]) / p;
	}
	// 1.00e30以上の終端の傾きが入力された場合は，終端は自然スプラインとする
	if (LastGradient > 0.99e30)
		qn = un = 0.0;
	else {
		qn = 0.5;
		un = (3.0 / (m_x[m_nodeNumber - 1] - m_x[m_nodeNumber - 2]))*(LastGradient - (m_y[m_nodeNumber - 1] - m_y[m_nodeNumber - 2]) / (m_x[m_nodeNumber - 1] - m_x[m_nodeNumber - 2]));
	}

	m_y2d[m_nodeNumber - 1] = (un - qn*u[m_nodeNumber - 2]) / (qn*m_y2d[m_nodeNumber - 2] + 1.0);

	for (j = m_nodeNumber - 2; j >= 0; j--)
		m_y2d[j] = m_y2d[j] * m_y2d[j + 1] + u[j];

	m_isInit = true;

	u.clear();
}


double CubicSpline::get(double time)
{
	int klo, khi, k;
	double h, b, a, want_y;


	if (!m_isInit) {
		return 0;
		//cout << "Not Spliend !!" << endl;
	}


	if ( (time - NEAR_ZERO) <= m_x.front() ) {
		//std::cout << "front" << std::endl;
		return m_y.front();
	}
	else if ( (time + NEAR_ZERO) >= m_x.back() ) {
		//std::cout << "back" << std::endl;
		return m_y.back();
	}


	klo = 0;
	khi = m_nodeNumber - 1;

	// 2分探索アルゴリズムループ　計算量：平均log2N回の試行
	while (khi - klo > 1) {
		k = (khi + klo) >> 1;
		if (m_x[k] > time) khi = k;
		else klo = k;
	}

	h = m_x[khi] - m_x[klo];

	// 見つからない場合
	if (h == 0.0) {
		//std::cout << "not found" << std::endl;
		return 0;
	}

	a = (m_x[khi] - time) / h;
	b = (time - m_x[klo]) / h;

	// 3次スプラインの補間値を計算
	want_y = a*m_y[klo] + b*m_y[khi] + ((a*a*a - a)*m_y2d[klo] + (b*b*b - b)*m_y2d[khi])*(h*h) / 6.0;

	return want_y;
}


void CubicSpline::sequence(std::vector<double> &seq, double sampling)
{
	int n = (int)std::ceil(( m_x.back() + NEAR_ZERO ) / sampling);
	double t0, t; 
	t0 = t = m_x.front();

	seq.clear();
	seq.push_back(m_y.front());
	for (int i = 1; i < n; i++) {
		t = t0 + i * sampling;
		seq.push_back(get(t));
	}
	seq.push_back(m_y.back());
}


vector<double> CubicSpline::sequence(double sampling)
{
	std::vector<double> seq;
	sequence(seq, sampling);
	return seq;
}
