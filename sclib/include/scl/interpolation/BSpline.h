#ifndef B_SPLINE_H
#define B_SPLINE_H

#include <vector>

class BSpline
{
public:
	BSpline(unsigned int dim = 3) { m_dim = dim;  m_searchType = false; }

	void setDim(unsigned int dim) { m_dim = dim; }

	// スプライン関数の計算 (実際にはgetの方がメイン・・・)
	//   Xsource : 横軸 (時間とか)
	//   Ysource : 縦軸 (位置とか)
	bool calc(const std::vector<double> &Xsource, const std::vector<double> &Ysource, const int knotType = 0);

	// 値の取得 (中でBスプラインの計算をしている)
	//   x = time
	//   y = f(x)
	double get(double time);

	// 初期化
	void clear();

	// コピーオペレータ
	BSpline& operator=(const BSpline& si);


private:
	void openKnot();     // 開ノットベクトルを計算 (時間を媒介変数にするために一様にはなってない)
	void uniformKnot();  // 一様ノットベクトルを計算

	// B-スプライン基底関数（B-spline basis function）
	// 変数名は wikipedia に合わせた ( b_i,i(t) )
	double bsplineBasisFunction(const unsigned int j, const unsigned int k, const double t, const std::vector<double>& knot);


	unsigned int m_dim;  // 補間式の次数
	int m_knotType;      // 使うノットベクトルのタイプ (0 : openKnot, 1 : uniformKnot)
	std::vector<double> m_knot;    // ノットベクトル
	std::vector<double> m_x, m_y;  // 補間するノード(x : 横軸, y : 縦軸)

	bool m_searchType;  // 基底関数の計算範囲 ( true : 二分岐探索, false : 単純前方探索 )
};

#endif // !B_SPLINE_H
