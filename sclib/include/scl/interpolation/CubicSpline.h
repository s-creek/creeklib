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

	void clear();  // 初期化
	bool isInit() { return m_isInit; }  // true : スプライン計算済み, false : 未計算

	// スプライン関数の計算
	void calc(const std::vector<double> &Xsource, const std::vector<double> &Ysource, double FirstGradient = 0, double LastGradient = 0);

	// 値の取得 (time = Xsource)
	double get(double time);

	// サンプリング時間で刻んだスプライン曲線の全値を取得
	void sequence(std::vector<double> &seq, double sampling);
	std::vector<double> sequence(double sampling);

	// コピーオペレータ
	CubicSpline& operator=(const CubicSpline& si);

	// 加速度のノード
	const std::vector<double>& accList() const { return m_y2d; }


private:
	bool m_isInit;  // true : スプライン計算済み, false : 未計算

	int m_nodeNumber;  // 補間するノードの数
	std::vector<double> m_x, m_y;  // 補間するノード(x : 横軸, y : 縦軸)
	std::vector<double> m_y2d;     // スプライン関数
};


#endif // ! CUBIC_SPLINE_H

