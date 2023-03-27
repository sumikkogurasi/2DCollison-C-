#include "stdafx.h"
#include "GUIObject.h"
#include <time.h>

// 追加した標準ライブラリ
#include <array>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

#define VECTOR_ADD_RECT_ROT_COMBINE_BALL

// 矩形に回転を追加出来なかったため三角形は諦めました。

// 矩形と円のコリジョン解決（未完成）
#ifdef VECTOR_ADD_RECT_ROT_COMBINE_BALL

// プロトタイプ宣言
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分のクラス
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // 垂線ベクトル
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// 線分のコンストラクタ
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	V = VectorSub(Sp, Ep);

	// 法線ベクトル
	N.x = -V.y;
	N.y = V.x;

	// それぞれ正規化
	VectorNormalize(N);
	VectorNormalize(V);
}

// 円のクラス
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
	float rate = 0.6f; // 衝突応答に使う係数
public:
	BALL(UINT color, float r, float px, float py, float restitution);

	const FXY& pos() { return Pos; }
	const float& radius() { return Radius; }

	// 加速度
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// 線分と円の中心との距離（内積）
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();

			// 垂線と線分の両点の外積
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {

				// めり込みした距離を計算
				float excess = Radius - abs(dist);

				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;

				calcReflect(n);
				Vel *= Restitution;
			}

			// 始点と円の中心との距離
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}

			// 終点と円の中心との距離
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// 円との衝突判定、応答
	void collisionBall(BALL& other) {
		if (this != &other) {
			FXY v = Pos - other.Pos;
			float dist = Radius + other.Radius;
			if (v.x * v.x + v.y * v.y < dist * dist) {
				float excess = dist - VectorLength(v);
				VectorNormalize(v);
				Pos += v * excess * rate;
				other.Pos -= v * excess * rate;
				calcReflect(v);
				Vel *= Restitution;
			}
		}
	}

	// 反射ベクトルを求める
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// 描画
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// 円のコンストラクタ
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// 矩形のクラス
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// 幅
	float h = 0.0f;		// 高さ
	float ag = 0.0f;	// 角度
	UINT Color;

	FXY point[4];	// rad = 0 時の各頂点の座標
	FXY Outpoint[4];	// rad = ag/pi 時の各頂点の座標

	float Restitution = 0.0f; // 反発係数

	float state = false;

	float dir = 0.0f; // 外積の符号確認用

	FXY Axis; // 物体に衝突した時に接触した点の位置を保管

	FXY N[4]; // 矩形の辺の法線ベクトル

	int num = 5;

	bool situation = false;

	FXY mv[4]; // 自分自身の辺のベクトル

	bool StaticRect = false; // コップと接触する点の数に応じて物体を固定するフラグ

	bool StaticRect2 = false; // 矩形と接触する点の数に応じて物体を固定するフラグ

	int count2 = 0; // コップとの接触点の数

	int count3 = 0; // 矩形との接触点の数

	FXY LOG; // 線分との衝突判定終了時の計算結果を保管

public:

	// コンストラクタ
	RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution);

	const FXY& OUTPOINT(int i) { return Outpoint[i]; }

	// 加速度
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 移動
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;

		ag = FIT_ANGLE(ag);

		// 四角形の形状から回転を考慮した頂点の位置を求める
		CalcPoint();

		count2 = 0;

		count3 = 0;

		StaticRect = false;

		StaticRect2 = false;

	}

	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {

		const FXY& n = seg.n();
		const FXY& v = seg.v();

		// 線分の始点から矩形の4頂点へのベクトル
		FXY v0 = Outpoint[0] - seg.sp();
		FXY v1 = Outpoint[1] - seg.sp();
		FXY v2 = Outpoint[2] - seg.sp();
		FXY v3 = Outpoint[3] - seg.sp();

		// 線分と矩形の頂点までの距離
		float dist0 = FXYDDot(n, v0);
		float dist1 = FXYDDot(n, v1);
		float dist2 = FXYDDot(n, v2);
		float dist3 = FXYDDot(n, v3);

		// 矩形の頂点が線分の左右どちらにあるか符号で判断する
		float value0 = FXYDCross(v, v0);
		float value1 = FXYDCross(v, v1);
		float value2 = FXYDCross(v, v2);
		float value3 = FXYDCross(v, v3);

		// 矩形の頂点がすべて線分に対して同じ方向にあった場合
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {

			state = true;

			// 線分の終点から矩形の4頂点へのベクトル
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();

			// 矩形の頂点から矩形の重心までのベクトル
			FXY centerV0 = Pos - Outpoint[0];
			FXY centerV1 = Pos - Outpoint[1];
			FXY centerV2 = Pos - Outpoint[2];
			FXY centerV3 = Pos - Outpoint[3];

			// 線分の法線ベクトルと矩形の頂点から矩形の重心までのベクトルの外積の符号を判定する用
			float valueN0 = FXYDCross(n, centerV0);
			float valueN1 = FXYDCross(n, centerV1);
			float valueN2 = FXYDCross(n, centerV2);
			float valueN3 = FXYDCross(n, centerV3);

			// 線分のベクトルと矩形の頂点から矩形の重心までのベクトルの外積の符号を判定する用
			float valueC0 = FXYDCross(v, centerV0);
			float valueC1 = FXYDCross(v, centerV1);
			float valueC2 = FXYDCross(v, centerV2);
			float valueC3 = FXYDCross(v, centerV3);

			if (value0 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v0) < 0 && FXYDCross(n, v4) > 0) {
					float excess2 = dist0;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[0];

					dir = valueN0;

					count3++;
				}
			}
			if (value1 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v1) < 0 && FXYDCross(n, v5) > 0) {
					float excess2 = dist1;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[1];

					dir = valueN1;

					count3++;
				}
			}
			if (value2 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v2) < 0 && FXYDCross(n, v6) > 0) {
					float excess2 = dist2;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[2];

					dir = valueN2;

					count3++;
				}
			}
			if (value3 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v3) < 0 && FXYDCross(n, v7) > 0) {
					float excess2 = abs(dist3);
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[3];

					dir = valueN3;

					count3++;
				}

			}

			LOG = Pos;

			// 線分と接触した頂点の数が二つ以上の場合物体を固定する
			if (count3 >= 2) {
				StaticRect = true;
			}

			// 固定されてなかったら回転させる
			if (!StaticRect) {
				if (value0 > 0) {
					RotPos(v, mv[0], mv[3], 1);
				}
				else if (value1 > 0) {
					RotPos(v, mv[1], mv[0], 1);
				}
				else if (value2 > 0) {
					RotPos(v, mv[2], mv[1], 1);
				}
				else if (value3 > 0) {
					RotPos(v, mv[3], mv[2], 1);
				}
			}

			//TRACE(_T("Seg count:%d\n"), count3);
			//TRACE(_T("Static Rect:%d\n"), StaticRect);
			//TRACE(_T("----------------\n"));
			//// 始点と円の中心との距離
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// 終点と円の中心との距離
			//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
			//	VectorNormalize(v2);
			//	Pos = seg.ep() + v2 * Radius;
			//	calcReflect(v2);
			//	Vel *= Restitution;
			//}

		}
	}

	// 矩形との衝突判定、応答
	void collisionRectangle(RECTANGLE& other) {
		if (this != &other) {

			// 接触した矩形の辺のベクトル
			FXY ov[4];
			ov[0] = other.Outpoint[1] - other.Outpoint[0];
			ov[1] = other.Outpoint[2] - other.Outpoint[1];
			ov[2] = other.Outpoint[3] - other.Outpoint[2];
			ov[3] = other.Outpoint[0] - other.Outpoint[3];

			// [相手][自分] 頂点へのベクトル
			FXY inside[4][4];
			for (int i = 0; i < 4; i++) {
				inside[0][i] = Outpoint[i] - other.Outpoint[0];
				inside[1][i] = Outpoint[i] - other.Outpoint[1];
				inside[2][i] = Outpoint[i] - other.Outpoint[2];
				inside[3][i] = Outpoint[i] - other.Outpoint[3];
			}


			// [相手][自分] 外積
			float value[4][4];
			for (int i = 0; i < 4; i++) {
				value[0][i] = FXYDCross(ov[0], inside[0][i]);
				value[1][i] = FXYDCross(ov[1], inside[1][i]);
				value[2][i] = FXYDCross(ov[2], inside[2][i]);
				value[3][i] = FXYDCross(ov[3], inside[3][i]);
			}

			// 自分の4つの頂点から自分自身の重心へのベクトル
			FXY centerV[4];

			for (int i = 0; i < 4; i++) {
				centerV[i] = Pos - Outpoint[i];
			}

			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("v[%d](x:%f, y%f)\n"), i, v[i].x, v[i].y);
			//}
			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("inside[%d][0](x:%f, y%f)\n"), i, inside[i][0].x, inside[i][0].y);
			//	TRACE(_T("inside[%d][1](x:%f, y%f)\n"), i, inside[i][1].x, inside[i][1].y);
			//	TRACE(_T("inside[%d][2](x:%f, y%f)\n"), i, inside[i][2].x, inside[i][2].y);
			//	TRACE(_T("inside[%d][3](x:%f, y%f)\n"), i, inside[i][3].x, inside[i][3].y);
			//}
			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("value[0][%d]:%f\n"), i, value[0][i]);
			//	TRACE(_T("value[1][%d]:%f\n"), i, value[1][i]);
			//	TRACE(_T("value[2][%d]:%f\n"), i, value[2][i]);
			//	TRACE(_T("value[3][%d]:%f\n"), i, value[3][i]);
			//	TRACE(_T("---------------------\n"));
			//}

			// 自分の頂点4つに対して衝突しているか判定する
			for (int i = 0; i < 4; i++) {
				if (value[0][i] <= 0 && value[1][i] <= 0 && value[2][i] <= 0 && value[3][i] <= 0) {

					situation = true;

					// 線分と頂点との距離が最小の線分の番号を記録するために置いた（要修正）
					int min = 0;

					for (int j = 0; j < 4; j++) {
						// 自分の線分と相手のどの線分と交差したかを判定
						if (CrossSegment(ov[j], -centerV[i], other.Outpoint[j], Pos)) {

							// その場合それぞれ接触点をカウントする
							count2++;
							other.count2++;
							min = j;
						}
					}

					// 相手の線分4つと相手の矩形に侵入した自分の頂点との距離
					float dist[4];

					// その計算
					for (int j = 0; j < 4; j++) {
						dist[j] = abs(FXYDDot(other.N[j], inside[j][i]));
					}

					//TRACE(_T("Inside value[0][%d]:%f\n"), i, value[0][i]);
					//TRACE(_T("Inside value[1][%d]:%f\n"), i, value[1][i]);
					//TRACE(_T("Inside value[2][%d]:%f\n"), i, value[2][i]);
					//TRACE(_T("Inside value[3][%d]:%f\n"), i, value[3][i]);
					//for (int i = 0; i < 4; i++) {
					//	TRACE(_T("dist[%d]:%f\n"), i, dist[i]);
					//}
					//TRACE(_T("min:%d\n"), min);

					// 矩形と接触し、回転するような場合に必要な接触した相手の線分の法線ベクトルと自分の頂点から自分の重心までのベクトルの外積の符号を判定するのに用いる
					float valueN[4];

					for (int i = 0; i < 4; i++) {
						valueN[i] = FXYDCross(-other.N[min], centerV[i]);
					}

					num = i;

					dir = valueN[i];

					// 
					RotPosCount(-ov[min], mv[i], mv[(i + 3) % 4]);

					// 加速度の逆ベクトル（要改善）
					FXY H;
					H.x = 0;
					H.y = -1;
					dir = FXYDCross(H, -centerV[i]);

					//other.Color = RGB(255, 255, 0);
					//Color = RGB(255, 255, 0);

					// 二つの矩形を比較して触れている頂点がそれぞれ2個以上の場合
					if (count2 + count3 < 2 && other.count2 + other.count3 < 2) {
						Pos += other.N[min] * dist[min] * 0.5f;
						other.Pos -= other.N[min] * dist[min] * 0.5f;

						//Color = RGB(0, 255, 255);
						//other.Color = RGB(0, 0, 0);

						Axis = Outpoint[i];
					}
					else if (count2 + count3 < 2 && other.count2 + other.count3 >= 2) {
						Pos += other.N[min] * dist[min] * 1.1f;

						//Color = RGB(0, 0, 255);

						Axis = Outpoint[i];

					}
					else if (count2 + count3 >= 2 && other.count2 + other.count3 < 2) {
						other.Pos -= other.N[min] * dist[min] * 1.1f;

						//other.Color = RGB(255, 255, 255);

						Axis = Outpoint[i];

					}
					else if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {

						if (Pos.y > other.Pos.y) {

							//other.Color = RGB(0, 255, 0);

							other.Pos -= other.N[min] * dist[min];
						}
						else if (Pos.y < other.Pos.y) {

							//Color = RGB(255, 0, 0);

							Pos += other.N[min] * dist[min];
						}

						Axis = Outpoint[i];
					}

					CalcPoint();

					//TRACE(_T("count3: %d\n"), count3);
					//TRACE(_T("count2: %d\n"), count2);
					//TRACE(_T("Sum: %d\n"), count3+ count2);
					//TRACE(_T("-----------------\n"));

					if (count2 + count3 < 2 && CenterRot(other, min, i)) {
						RotPos(-ov[min], mv[i], mv[(i + 3) % 4], 1);
					}

					calcReflect(N[min]);
					Vel *= Restitution;

					CalcPoint();

					//// 始点と円の中心との距離
					//
					//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
					//	VectorNormalize(v1);
					//	Pos = seg.sp() + v1 * Radius;
					//	calcReflect(v1);
					//	Vel *= Restitution;
					//}
					//// 終点と円の中心との距離
					//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
					//	VectorNormalize(v2);
					//	Pos = seg.ep() + v2 * Radius;
					//	calcReflect(v2);
					//	Vel *= Restitution;
					//}

				}


			}
		}
	}

	// 円との衝突判定、応答
	void collisionBall(BALL& ball) {
		const FXY& p = ball.pos();
		const float& r = ball.radius();

		// [相手][自分] 頂点へのベクトル
		FXY inside[4];
		for (int i = 0; i < 4; i++) {

		}
	}

	// 重心が辺の外に出ているか
	bool CenterRot(RECTANGLE& other, int min, int i) {

		// 物体同士が固定していた場合
		if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {
			TRACE(_T("CenterRot ME Sum: %d\n"), count3 + count2);
			TRACE(_T("CenterRot OTHER Sum: %d\n"), other.count2 + other.count3);
			return false;
		}

		// 自分が相手の上に重なる場合で自分の重心が相手の線分の両端の位置からはみ出していた場合
		if (count2 + count3 < 2) {
			if (Pos.x > other.Outpoint[min].x || Pos.x < other.Outpoint[(min + 1) % 4].x) {
				if (Outpoint[(i + 1) % 4].y - other.Outpoint[min].y < 0 || Outpoint[(i + 3) % 4].y - other.Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// 相手が自分の上に重なる場合で相手の重心が自分の線分の両端の位置からはみ出していた場合
		if (other.count2 + other.count3 < 2) {
			if (other.Pos.x > Outpoint[min].x || other.Pos.x < Outpoint[(min + 1) % 4].x) {
				if (other.Outpoint[(i + 1) % 4].y - Outpoint[min].y < 0 || other.Outpoint[(i + 3) % 4].y - Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// 不完全
		return true;
	}

	// ２つの線分が交わっているか判定
	bool CrossSegment(FXY V1, FXY V2, FXY S1, FXY S2) {
		FXY v = S2 - S1;
		float CrsV1V2 = FXYDCross(V1, V2);

		if (CrsV1V2 == 0.0f) {
			return false;
		}

		float CrsvV1 = FXYDCross(v, V1);
		float CrsvV2 = FXYDCross(v, V2);

		float t1 = CrsvV1 / CrsV1V2;
		float t2 = CrsvV2 / CrsV1V2;

		// 0 <= t <=1の場合
		if (t1 < 0 || t1 > 1 || t2 < 0 || t2 > 1) {
			return false;
		}

		return true;
	}

	// 反射ベクトルを計算
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// 配列内で最小値がある番号を返す
	int minValue(float nums[], int n) {
		float min_value; // 最大値
		int i;
		int j = 0;
		// nums[0]を最大値と仮定する
		min_value = nums[0];

		for (i = 0; i < n; i++) {
			if (nums[i] < min_value) {
				/* 最大値よりもnums[i]の方が大きければ最大値を更新 */
				min_value = nums[i];
				j = i;
			}
		}

		return j;
	}

	// 矩形と接触して回転した後相手の線分に接近した場合接触点を加える
	// V　　接触した相手のベクトル
	// V1　 接触した頂点の左側のベクトル
	// V2　 接触した頂点の右側のベクトル

	void RotPosCount(FXY V, FXY V1, FXY V2)
	{
		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// 自分の重心が相手の法線の左側にあった場合
		if (dir > 0) {

			// 相手の線分と自分の線分のベクトルが平行に近づいた場合
			if (abs(FXYDDot(V, -V2)) < 0.1f) {
				count2++;
			}
		}

		// 相手の重心が自分の法線の右側にあった場合
		else if (dir < 0) {

			// 相手の線分と自分の線分のベクトルが平行に近づいた場合
			if (abs(FXYDDot(-V, V1)) < 0.1f) {
				count2++;
			}
		}
	}

	// 線分と接したときの回転後の座標取得.
	// V　　接触した相手のベクトル
	// V1　 接触した頂点の左側のベクトル
	// V2　 接触した頂点の右側のベクトル
	// fAngle	回転角度.

	void RotPos(FXY V, FXY V1, FXY V2, float fAngle)
	{
		if (!IS_FIT_ANGLE(fAngle)) {
			TRACE(_T("角度が0.0～360.0に収まっていません[%f]\n"), fAngle);
			ASSERT(false);	// FIT_ANGLE()を通して戻された値を次回以降も利用する必要があります.
		}

		float rad = 0;	// ラジアン.
		float cosValue;
		float sinValue;

		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// 自分の重心が相手の法線の左側にあった場合
		if (dir > 0) {
			if (FXYDDot(V, -V2) < 0) {
				ag = ag - fAngle;
				rad = -ANGLE(fAngle);
			}
		}

		// 相手の重心が自分の法線の右側にあった場合
		else if (dir < 0) {
			if (FXYDDot(V, V1) > 0) {
				ag = ag + fAngle;
				rad = ANGLE(fAngle);
			}
		}

		cosValue = cosf(rad);
		sinValue = sinf(rad);

		// 回転変換
		Pos.x = Axis.x + (Pos.x - Axis.x) * cosValue - (Pos.y - Axis.y) * sinValue;
		Pos.y = Axis.y + (Pos.x - Axis.x) * sinValue + (Pos.y - Axis.y) * cosValue;

		ag = FIT_ANGLE(ag);
	}

	// 回転後の頂点の座標を求める
	void CalcPoint() {

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

		GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);

		mv[0] = Outpoint[1] - Outpoint[0];
		mv[1] = Outpoint[2] - Outpoint[1];
		mv[2] = Outpoint[3] - Outpoint[2];
		mv[3] = Outpoint[0] - Outpoint[3];

		for (int i = 0; i < 4; i++) {
			N[i].x = -mv[i].y;
			N[i].y = mv[i].x;
			VectorNormalize(N[i]);
		}
	}

	// 描画
	void draw(HDC& hdc) {

		RenderRotRectangle(hdc, Pos, point, ag, Color);
		RenderCircle(hdc, Pos.x, Pos.y, 5, Color);
		if (situation) {
			RenderCircle(hdc, Outpoint[num].x, Outpoint[num].y, 5, Color);
		}
	}
};

// 矩形のコンストラクタ
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// 幅
	h = height;		// 高さ

	ag = angle;	// 角度(45度回す).
	ag = FIT_ANGLE(ag);

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

	GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 10;	// 0.1秒間隔で更新.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// 各種オブジェクトの初期配置.
// 「初期配置リセット」のボタンを押したらこの関数が呼ばれます.
// その後、UpdateObject()がunTimer == 0 で１回だけ呼ばれます.
// 各図形の初期位置をこちらで設定してください.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	// 線分の始点と終点（コップ）
	{
		float p[] = {
		g_sCupPoint[0].x, g_sCupPoint[0].y,
		g_sCupPoint[1].x, g_sCupPoint[1].y,
		g_sCupPoint[2].x, g_sCupPoint[2].y,
		g_sCupPoint[3].x, g_sCupPoint[3].y,
		};

		segments.clear();

		int numLine = sizeof(p) / 8 - 1;
		for (int i = 0; i < numLine; i++) {
			segments.emplace_back(
				p[i * 2 + 0],
				p[i * 2 + 1],
				p[i * 2 + 2],
				p[i * 2 + 3]
			);
		}
	}

	// 矩形の初期化
	{
		// rects.clear();

		for (int i = 0; i < 1; i++) {

			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float width = RandomRangeF(50.0f, 150.0f);
			float height = RandomRangeF(50.0f, 150.0f);
			float angle = RandomRangeF(0.0f, 360.0f);
			//float width = 100;
			//float height = 50;
			//float angle = 45.0f;
			float restitution = 0.0f;

			rects.emplace_back(rgb, px, py, width, height, angle, restitution);
		}
	}


	// 円の初期化
	{
		//balls.clear();

		for (int i = 0; i < 10; i++) {

			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float radius = RandomRangeF(20.0f, 50.0f);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float restitution = 0.5f;

			balls.emplace_back(rgb, radius, px, py, restitution);
		}
	}
}

void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset, HDC& hdc);


// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(&bReset, hdc);

}

// 位置更新
void UpdatePos(bool* PtrbReset, HDC& hdc) {

	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		FXY gravity(0, 0.1f);

		// 円の位置更新
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity);
				ball.move();
				for (BALL& other : balls) {
					ball.collisionBall(other);
				}
				for (SEGMENT& segment : segments) {
					ball.collisionSegment(segment);
				}
				ball.draw(hdc);
			}

		}

		// 矩形の位置更新
		{
			for (RECTANGLE& rect : rects) {
				rect.applyForce(gravity);
				rect.move();

				for (SEGMENT& segment : segments) {
					rect.collisionSegment(segment);
				}
				for (RECTANGLE& other : rects) {
					rect.collisionRectangle(other);
				}
				rect.draw(hdc);
			}

		}
	}

}
// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}

#endif

// 矩形に回転を追加（未完成）　※発表に使用
#ifdef VECTOR_ADD_RECT_ROT

// プロトタイプ宣言
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分のクラス
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // 垂線ベクトル
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// 線分のコンストラクタ
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	V = VectorSub(Sp, Ep);

	// 法線ベクトル
	N.x = -V.y;
	N.y = V.x;

	// それぞれ正規化
	VectorNormalize(N);
	VectorNormalize(V);
}

// 円のクラス
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
	float rate = 0.6f; // 衝突応答に使う係数
public:
	BALL(UINT color, float r, float px, float py, float restitution);

	// 加速度
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// 線分と円の中心との距離（内積）
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();

			// 垂線と線分の両点の外積
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {

				// めり込みした距離を計算
				float excess = Radius - abs(dist);

				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;

				calcReflect(n);
				Vel *= Restitution;
			}

			// 始点と円の中心との距離
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}

			// 終点と円の中心との距離
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// 円との衝突判定、応答
	void collisionBall(BALL& other) {
		if (this != &other) {
			FXY v = Pos - other.Pos;
			float dist = Radius + other.Radius;
			if (v.x * v.x + v.y * v.y < dist * dist) {
				float excess = dist - VectorLength(v);
				VectorNormalize(v);
				Pos += v * excess * rate;
				other.Pos -= v * excess * rate;
				calcReflect(v);
				Vel *= Restitution;
			}
		}
	}

	// 反射ベクトルを求める
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// 描画
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// 円のコンストラクタ
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// 矩形のクラス
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// 幅
	float h = 0.0f;		// 高さ
	float ag = 0.0f;	// 角度
	UINT Color;

	FXY point[4];	// rad = 0 時の各頂点の座標
	FXY Outpoint[4];	// rad = ag/pi 時の各頂点の座標

	float Restitution = 0.0f; // 反発係数

	float state = false;

	float dir = 0.0f; // 外積の符号確認用

	FXY Axis; // 物体に衝突した時に接触した点の位置を保管

	FXY N[4]; // 矩形の辺の法線ベクトル

	int num = 5;

	bool situation = false;

	FXY mv[4]; // 自分自身の辺のベクトル

	bool StaticRect = false; // コップと接触する点の数に応じて物体を固定するフラグ

	bool StaticRect2 = false; // 矩形と接触する点の数に応じて物体を固定するフラグ

	int count2 = 0; // コップとの接触点の数

	int count3 = 0; // 矩形との接触点の数

	FXY LOG; // 線分との衝突判定終了時の計算結果を保管

public:

	// コンストラクタ
	RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution);

	const FXY& OUTPOINT(int i) { return Outpoint[i]; }

	// 加速度
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 移動
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;

		ag = FIT_ANGLE(ag);

		// 四角形の形状から回転を考慮した頂点の位置を求める
		CalcPoint();

		count2 = 0;

		count3 = 0;

		StaticRect = false;

		StaticRect2 = false;

	}

	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {

		const FXY& n = seg.n();
		const FXY& v = seg.v();

		// 線分の始点から矩形の4頂点へのベクトル
		FXY v0 = Outpoint[0] - seg.sp();
		FXY v1 = Outpoint[1] - seg.sp();
		FXY v2 = Outpoint[2] - seg.sp();
		FXY v3 = Outpoint[3] - seg.sp();

		// 線分と矩形の頂点までの距離
		float dist0 = FXYDDot(n, v0);
		float dist1 = FXYDDot(n, v1);
		float dist2 = FXYDDot(n, v2);
		float dist3 = FXYDDot(n, v3);

		// 矩形の頂点が線分の左右どちらにあるか符号で判断する
		float value0 = FXYDCross(v, v0);
		float value1 = FXYDCross(v, v1);
		float value2 = FXYDCross(v, v2);
		float value3 = FXYDCross(v, v3);

		// 矩形の頂点がすべて線分に対して同じ方向にあった場合
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {

			state = true;

			// 線分の終点から矩形の4頂点へのベクトル
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();
			
			// 矩形の頂点から矩形の重心までのベクトル
			FXY centerV0 = Pos - Outpoint[0];
			FXY centerV1 = Pos - Outpoint[1];
			FXY centerV2 = Pos - Outpoint[2];
			FXY centerV3 = Pos - Outpoint[3];

			// 線分の法線ベクトルと矩形の頂点から矩形の重心までのベクトルの外積の符号を判定する用
			float valueN0 = FXYDCross(n, centerV0);
			float valueN1 = FXYDCross(n, centerV1);
			float valueN2 = FXYDCross(n, centerV2);
			float valueN3 = FXYDCross(n, centerV3);

			// 線分のベクトルと矩形の頂点から矩形の重心までのベクトルの外積の符号を判定する用
			float valueC0 = FXYDCross(v, centerV0);
			float valueC1 = FXYDCross(v, centerV1);
			float valueC2 = FXYDCross(v, centerV2);
			float valueC3 = FXYDCross(v, centerV3);

			if (value0 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v0) < 0 && FXYDCross(n, v4) > 0) {
					float excess2 = dist0;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[0];

					dir = valueN0;

					count3++;
				}
			}
			if (value1 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v1) < 0 && FXYDCross(n, v5) > 0) {
					float excess2 = dist1;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[1];

					dir = valueN1;

					count3++;
				}
			}
			if (value2 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v2) < 0 && FXYDCross(n, v6) > 0) {
					float excess2 = dist2;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[2];

					dir = valueN2;

					count3++;
				}
			}
			if (value3 > 0) {

				// 垂線と線分の両点の外積
				if (FXYDCross(n, v3) < 0 && FXYDCross(n, v7) > 0) {
					float excess2 = abs(dist3);
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;

					CalcPoint();

					Axis = Outpoint[3];

					dir = valueN3;

					count3++;
				}

			}

			LOG = Pos;

			// 線分と接触した頂点の数が二つ以上の場合物体を固定する
			if (count3 >= 2) {
				StaticRect = true;
			}

			// 固定されてなかったら回転させる
			if (!StaticRect) {
				if (value0 > 0) {
					RotPos(v, mv[0], mv[3], 1);
				}
				else if (value1 > 0) {
					RotPos(v, mv[1], mv[0], 1);
				}
				else if (value2 > 0) {
					RotPos(v, mv[2], mv[1], 1);
				}
				else if (value3 > 0) {
					RotPos(v, mv[3], mv[2], 1);
				}
			}

			//TRACE(_T("Seg count:%d\n"), count3);
			//TRACE(_T("Static Rect:%d\n"), StaticRect);
			//TRACE(_T("----------------\n"));
			//// 始点と円の中心との距離
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// 終点と円の中心との距離
			//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
			//	VectorNormalize(v2);
			//	Pos = seg.ep() + v2 * Radius;
			//	calcReflect(v2);
			//	Vel *= Restitution;
			//}

		}
	}

	// 矩形との衝突判定、応答
	void collisionRectangle(RECTANGLE& other) {
		if (this != &other) {

			// 接触した矩形の辺のベクトル
			FXY ov[4];
			ov[0] = other.Outpoint[1] - other.Outpoint[0];
			ov[1] = other.Outpoint[2] - other.Outpoint[1];
			ov[2] = other.Outpoint[3] - other.Outpoint[2];
			ov[3] = other.Outpoint[0] - other.Outpoint[3];

			// [相手][自分] 頂点へのベクトル
			FXY inside[4][4];
			for (int i = 0; i < 4; i++) {
				inside[0][i] = Outpoint[i] - other.Outpoint[0];
				inside[1][i] = Outpoint[i] - other.Outpoint[1];
				inside[2][i] = Outpoint[i] - other.Outpoint[2];
				inside[3][i] = Outpoint[i] - other.Outpoint[3];
			}


			// [相手][自分] 外積
			float value[4][4];
			for (int i = 0; i < 4; i++) {
				value[0][i] = FXYDCross(ov[0],inside[0][i]);
				value[1][i] = FXYDCross(ov[1],inside[1][i]);
				value[2][i] = FXYDCross(ov[2],inside[2][i]);
				value[3][i] = FXYDCross(ov[3],inside[3][i]);
			}

			// 自分の4つの頂点から自分自身の重心へのベクトル
			FXY centerV[4];

			for (int i = 0; i < 4; i++) {
				centerV[i] = Pos - Outpoint[i];
			}
			
			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("v[%d](x:%f, y%f)\n"), i, v[i].x, v[i].y);
			//}
			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("inside[%d][0](x:%f, y%f)\n"), i, inside[i][0].x, inside[i][0].y);
			//	TRACE(_T("inside[%d][1](x:%f, y%f)\n"), i, inside[i][1].x, inside[i][1].y);
			//	TRACE(_T("inside[%d][2](x:%f, y%f)\n"), i, inside[i][2].x, inside[i][2].y);
			//	TRACE(_T("inside[%d][3](x:%f, y%f)\n"), i, inside[i][3].x, inside[i][3].y);
			//}
			//for (int i = 0; i < 4; i++) {
			//	TRACE(_T("value[0][%d]:%f\n"), i, value[0][i]);
			//	TRACE(_T("value[1][%d]:%f\n"), i, value[1][i]);
			//	TRACE(_T("value[2][%d]:%f\n"), i, value[2][i]);
			//	TRACE(_T("value[3][%d]:%f\n"), i, value[3][i]);
			//	TRACE(_T("---------------------\n"));
			//}

			// 自分の頂点4つに対して衝突しているか判定する
			for (int i = 0; i < 4; i++) {
				if (value[0][i] <= 0 && value[1][i] <= 0 && value[2][i] <= 0 && value[3][i] <= 0) {

					situation = true;

					// 線分と頂点との距離が最小の線分の番号を記録するために置いた（要修正）
					int min = 0;

					for (int j = 0; j < 4; j++) {
						// 自分の線分と相手のどの線分と交差したかを判定
						if (CrossSegment(ov[j], -centerV[i], other.Outpoint[j], Pos)) {

							// その場合それぞれ接触点をカウントする
							count2++;
							other.count2++;
							min = j;
						}
					}

					// 相手の線分4つと相手の矩形に侵入した自分の頂点との距離
					float dist[4];
					
					// その計算
					for (int j = 0; j < 4; j++) {
						dist[j] = abs(FXYDDot(other.N[j], inside[j][i]));
					}

					//TRACE(_T("Inside value[0][%d]:%f\n"), i, value[0][i]);
					//TRACE(_T("Inside value[1][%d]:%f\n"), i, value[1][i]);
					//TRACE(_T("Inside value[2][%d]:%f\n"), i, value[2][i]);
					//TRACE(_T("Inside value[3][%d]:%f\n"), i, value[3][i]);
					//for (int i = 0; i < 4; i++) {
					//	TRACE(_T("dist[%d]:%f\n"), i, dist[i]);
					//}
					//TRACE(_T("min:%d\n"), min);

					// 矩形と接触し、回転するような場合に必要な接触した相手の線分の法線ベクトルと自分の頂点から自分の重心までのベクトルの外積の符号を判定するのに用いる
					float valueN[4];

					for (int i = 0; i < 4; i++) {
						valueN[i] = FXYDCross(-other.N[min], centerV[i]);
					}

					num = i;

					dir = valueN[i];

					// 
					RotPosCount(-ov[min], mv[i], mv[(i + 3) % 4]);

					// 加速度の逆ベクトル（要改善）
					FXY H;
					H.x = 0;
					H.y = -1;
					dir = FXYDCross(H, -centerV[i]);

					//other.Color = RGB(255, 255, 0);
					//Color = RGB(255, 255, 0);
					
					// 二つの矩形を比較して触れている頂点がそれぞれ2個以上の場合
					if (count2 + count3 < 2 && other.count2 + other.count3 < 2) {
						Pos += other.N[min] * dist[min] * 0.5f;
						other.Pos -= other.N[min] * dist[min] * 0.5f;

						//Color = RGB(0, 255, 255);
						//other.Color = RGB(0, 0, 0);

						Axis = Outpoint[i];
					}
					else if(count2 + count3 < 2 && other.count2 + other.count3 >= 2) {
						Pos += other.N[min] * dist[min] * 1.1f;

						//Color = RGB(0, 0, 255);

						Axis = Outpoint[i];

					}
					else if (count2 + count3 >= 2 && other.count2 + other.count3 < 2) {
						other.Pos -= other.N[min] * dist[min] * 1.1f;

						//other.Color = RGB(255, 255, 255);
					
						Axis = Outpoint[i];

					}
					else if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {
						
						if (Pos.y > other.Pos.y) {

							//other.Color = RGB(0, 255, 0);

							other.Pos -= other.N[min] * dist[min];
						}
						else if (Pos.y < other.Pos.y) {

							//Color = RGB(255, 0, 0);

							Pos += other.N[min] * dist[min];
						}

						Axis = Outpoint[i];
					}

					CalcPoint();

					//TRACE(_T("count3: %d\n"), count3);
					//TRACE(_T("count2: %d\n"), count2);
					//TRACE(_T("Sum: %d\n"), count3+ count2);
					//TRACE(_T("-----------------\n"));

					if (count2 + count3 < 2 && CenterRot(other, min, i)) {
							RotPos(-ov[min], mv[i], mv[(i + 3) % 4], 1);
					}

					calcReflect(N[min]);
					Vel *= Restitution;

					CalcPoint();

					//// 始点と円の中心との距離
					//
					//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
					//	VectorNormalize(v1);
					//	Pos = seg.sp() + v1 * Radius;
					//	calcReflect(v1);
					//	Vel *= Restitution;
					//}
					//// 終点と円の中心との距離
					//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
					//	VectorNormalize(v2);
					//	Pos = seg.ep() + v2 * Radius;
					//	calcReflect(v2);
					//	Vel *= Restitution;
					//}

				}


			}
		}
	}

	// 重心が辺の外に出ているか
	bool CenterRot(RECTANGLE& other, int min, int i) {

		// 物体同士が固定していた場合
		if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {
			TRACE(_T("CenterRot ME Sum: %d\n"), count3 + count2);
			TRACE(_T("CenterRot OTHER Sum: %d\n"), other.count2 + other.count3);
			return false;
		}

		// 自分が相手の上に重なる場合で自分の重心が相手の線分の両端の位置からはみ出していた場合
		if (count2 + count3 < 2) {
			if (Pos.x > other.Outpoint[min].x || Pos.x < other.Outpoint[(min + 1) % 4].x) {
				if (Outpoint[(i + 1) % 4].y - other.Outpoint[min].y < 0 || Outpoint[(i + 3) % 4].y - other.Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// 相手が自分の上に重なる場合で相手の重心が自分の線分の両端の位置からはみ出していた場合
		if (other.count2 + other.count3 < 2) {
			if (other.Pos.x > Outpoint[min].x || other.Pos.x < Outpoint[(min + 1) % 4].x) {
				if (other.Outpoint[(i + 1) % 4].y - Outpoint[min].y < 0 || other.Outpoint[(i + 3) % 4].y - Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// 不完全
		return true;
	}

	// ２つの線分が交わっているか判定
	bool CrossSegment(FXY V1, FXY V2, FXY S1, FXY S2) {
		FXY v = S2 - S1;
		float CrsV1V2 = FXYDCross(V1, V2);

		if (CrsV1V2 == 0.0f) {
			return false;
		}

		float CrsvV1 = FXYDCross(v, V1);
		float CrsvV2 = FXYDCross(v, V2);

		float t1 = CrsvV1 / CrsV1V2;
		float t2 = CrsvV2 / CrsV1V2;

		// 0 <= t <=1の場合
		if (t1 < 0 || t1 > 1 || t2 < 0 || t2 > 1) {
			return false;
		}

		return true;
	}

	// 反射ベクトルを計算
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// 配列内で最小値がある番号を返す
	int minValue(float nums[], int n) {
		float min_value; // 最大値
		int i;
		int j = 0;
		// nums[0]を最大値と仮定する
		min_value = nums[0];

		for (i = 0; i < n; i++) {
			if (nums[i] < min_value) {
				/* 最大値よりもnums[i]の方が大きければ最大値を更新 */
				min_value = nums[i];
				j = i;
			}
		}

		return j;
	}

	// 矩形と接触して回転した後相手の線分に接近した場合接触点を加える
	// V　　接触した相手のベクトル
	// V1　 接触した頂点の左側のベクトル
	// V2　 接触した頂点の右側のベクトル

	void RotPosCount(FXY V, FXY V1, FXY V2)
	{
		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// 自分の重心が相手の法線の左側にあった場合
		if (dir > 0) {

			// 相手の線分と自分の線分のベクトルが平行に近づいた場合
			if (abs(FXYDDot(V, -V2)) < 0.1f) {
				count2++;
			}
		}

		// 相手の重心が自分の法線の右側にあった場合
		else if (dir < 0) {

			// 相手の線分と自分の線分のベクトルが平行に近づいた場合
			if (abs(FXYDDot(-V,  V1)) < 0.1f) {
				count2++;
			}
		}
	}

	// 線分と接したときの回転後の座標取得.
	// V　　接触した相手のベクトル
	// V1　 接触した頂点の左側のベクトル
	// V2　 接触した頂点の右側のベクトル
	// fAngle	回転角度.

	void RotPos(FXY V, FXY V1, FXY V2, float fAngle)
	{
		if (!IS_FIT_ANGLE(fAngle)) {
			TRACE(_T("角度が0.0～360.0に収まっていません[%f]\n"), fAngle);
			ASSERT(false);	// FIT_ANGLE()を通して戻された値を次回以降も利用する必要があります.
		}

		float rad = 0;	// ラジアン.
		float cosValue;
		float sinValue;

		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// 自分の重心が相手の法線の左側にあった場合
		if (dir > 0) {
			if (FXYDDot(V, -V2) < 0) {
				ag = ag - fAngle;
				rad = -ANGLE(fAngle);
			}
		}

		// 相手の重心が自分の法線の右側にあった場合
		else if(dir < 0) {
			if (FXYDDot(V, V1) > 0) {
				ag = ag + fAngle;
				rad = ANGLE(fAngle);
			}
		}

		cosValue = cosf(rad);
		sinValue = sinf(rad);

		// 回転変換
		Pos.x = Axis.x + (Pos.x - Axis.x) * cosValue - (Pos.y - Axis.y) * sinValue;
		Pos.y = Axis.y + (Pos.x - Axis.x) * sinValue + (Pos.y - Axis.y) * cosValue;

		ag = FIT_ANGLE(ag);
	}

	// 回転後の頂点の座標を求める
	void CalcPoint() {

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

		GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);

		mv[0] = Outpoint[1] - Outpoint[0];
		mv[1] = Outpoint[2] - Outpoint[1];
		mv[2] = Outpoint[3] - Outpoint[2];
		mv[3] = Outpoint[0] - Outpoint[3];

		for (int i = 0; i < 4; i++) {
			N[i].x = -mv[i].y;
			N[i].y = mv[i].x;
			VectorNormalize(N[i]);
		}
	}

	// 描画
	void draw(HDC& hdc) {

		RenderRotRectangle(hdc, Pos, point, ag, Color);
		RenderCircle(hdc, Pos.x, Pos.y, 5, Color);
		if (situation) {
			RenderCircle(hdc, Outpoint[num].x, Outpoint[num].y, 5, Color);
		}
	}
};

// 矩形のコンストラクタ
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// 幅
	h = height;		// 高さ

	ag = angle;	// 角度(45度回す).
	ag = FIT_ANGLE(ag);

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

	GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 10;	// 0.1秒間隔で更新.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// 各種オブジェクトの初期配置.
// 「初期配置リセット」のボタンを押したらこの関数が呼ばれます.
// その後、UpdateObject()がunTimer == 0 で１回だけ呼ばれます.
// 各図形の初期位置をこちらで設定してください.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	// 線分の始点と終点（コップ）
	{
		float p[] = {
		g_sCupPoint[0].x, g_sCupPoint[0].y,
		g_sCupPoint[1].x, g_sCupPoint[1].y,
		g_sCupPoint[2].x, g_sCupPoint[2].y,
		g_sCupPoint[3].x, g_sCupPoint[3].y,
		};

		segments.clear();

		int numLine = sizeof(p) / 8 - 1;
		for (int i = 0; i < numLine; i++) {
			segments.emplace_back(
				p[i * 2 + 0],
				p[i * 2 + 1],
				p[i * 2 + 2],
				p[i * 2 + 3]
			);
		}
	}

	// 矩形の初期化
	{
		// rects.clear();

		for (int i = 0; i < 1; i++) {

			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float width = RandomRangeF(50.0f, 150.0f);
			float height = RandomRangeF(50.0f, 150.0f);
			float angle = RandomRangeF(0.0f, 360.0f);
			//float width = 100;
			//float height = 50;
			//float angle = 45.0f;
			float restitution = 0.0f;

			rects.emplace_back(rgb, px, py, width, height, angle, restitution);
		}
	}

	
	// 円の初期化
	//{
	//	//balls.clear();

	//	for (int i = 0; i < 10; i++) {

	//		UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
	//		float radius = RandomRangeF(20.0f, 50.0f);
	//		float px = RandomRangeF(200.0f, 500.0f);
	//		float py = RandomRangeF(100.0f, 200.0f);
	//		float restitution = 0.5f;

	//		balls.emplace_back(rgb, radius, px, py, restitution);
	//	}
	//}
}

void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset, HDC& hdc);


// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(&bReset, hdc);

}
// 位置更新
void UpdatePos(bool* PtrbReset, HDC& hdc) {

	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		FXY gravity(0, 0.1f);

		// 円の位置更新
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity); // 加速度
				ball.move(); // 移動
				for (BALL& other : balls) {
					ball.collisionBall(other); // 円との衝突判定
				}
				for (SEGMENT& segment : segments) {
					ball.collisionSegment(segment); // 線分との衝突判定
				}
				ball.draw(hdc); // 描画
			}
		}

		// 矩形の位置更新
		{
			for (RECTANGLE& rect : rects) {
				rect.applyForce(gravity);  // 加速度
				rect.move();  // 移動

				for (SEGMENT& segment : segments) {
					rect.collisionSegment(segment);  // 線分との衝突判定
				}
				for (RECTANGLE& other : rects) {
					rect.collisionRectangle(other); // 矩形との衝突判定
				}
				rect.draw(hdc); // 描画
			}
		}

	}
}

// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}

#endif

// 矩形とコップとのコリジョン解決
#ifdef VECTOR_ADD_RECT
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分のクラス
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // 垂線ベクトル
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// 線分のコンストラクタ
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	//Ep - Sp;
	V = VectorSub(Sp, Ep);
	//FXY v = Ep - Sp;
	N.x = -V.y;
	N.y = V.x;
	VectorNormalize(N);
	VectorNormalize(V);
}

// 円のクラス
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
	float rate = 0.6f;
public:
	BALL(UINT color, float r, float px, float py, float restitution);

	void applyForce(const FXY& force) {
		Acc += force;
	}

	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// 線分と円の中心との距離（内積）
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();
			// 垂線と線分の両点の外積
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {
				float excess = Radius - abs(dist);
				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;
				calcReflect(n);
				Vel *= Restitution;
			}
			// 始点と円の中心との距離
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}
			// 終点と円の中心との距離
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// 円との衝突判定、応答
	void collisionBall(BALL& other) {
		if (this != &other) {
			FXY v = Pos - other.Pos;
			float dist = Radius + other.Radius;
			if (v.x * v.x + v.y * v.y < dist * dist) {
				float excess = dist - VectorLength(v);
				VectorNormalize(v);
				Pos += v * excess * rate;
				other.Pos -= v * excess * rate;
				calcReflect(v);
				Vel *= Restitution;
			}
		}
	}
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// 円のコンストラクタ
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// 矩形のクラス
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// 幅
	float h = 0.0f;		// 高さ
	float ag = 0.0f;	// 角度
	UINT Color;

	FXY point[4];	// rad = 0 時の各頂点の座標
	FXY Outpoint[4];	// rad = ag/pi 時の各頂点の座標

	float Restitution = 0.0f;

public:
	RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution);

	void applyForce(const FXY& force) {
		Acc += force;
	}

	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;

		ag = FIT_ANGLE(ag);

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

		GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);
		//point[0].x = point[0].x * cos(DEG2RAD(ag)) - point[0].y * sin(DEG2RAD(ag));
		//point[0].y = point[0].x * sin(DEG2RAD(ag)) + point[0].y * cos(DEG2RAD(ag));

		//point[1].x = point[1].x * cos(DEG2RAD(ag)) - point[1].y * sin(DEG2RAD(ag));
		//point[1].y = point[1].x * sin(DEG2RAD(ag)) + point[1].y * cos(DEG2RAD(ag));

		//point[2].x = point[2].x * cos(DEG2RAD(ag)) - point[2].y * sin(DEG2RAD(ag));
		//point[2].y = point[2].x * sin(DEG2RAD(ag)) + point[2].y * cos(DEG2RAD(ag));

		//point[3].x = point[3].x * cos(DEG2RAD(ag)) - point[3].y * sin(DEG2RAD(ag));
		//point[3].y = point[3].x * sin(DEG2RAD(ag)) + point[3].y * cos(DEG2RAD(ag));
	}

	int num = 0;
	// 線分との衝突判定、応答
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		const FXY& v = seg.v();
		FXY v0 = Outpoint[0] - seg.sp();
		FXY v1 = Outpoint[1] - seg.sp();
		FXY v2 = Outpoint[2] - seg.sp();
		FXY v3 = Outpoint[3] - seg.sp();
		float dist0 = FXYDDot(n, v0);
		float dist1 = FXYDDot(n, v1);
		float dist2 = FXYDDot(n, v2);
		float dist3 = FXYDDot(n, v3);

		float value0 = FXYDCross(v, v0);
		float value1 = FXYDCross(v, v1);
		float value2 = FXYDCross(v, v2);
		float value3 = FXYDCross(v, v3);

		//TRACE(_T("d0: %f \n"), dist0);
		//TRACE(_T("d1: %f \n"), dist1);
		//TRACE(_T("value1: %f \n"), value1);

		// 線分と円の中心との距離（内積）
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();

			if (value0 > 0) {
				// 垂線と線分の両点の外積
				if (FXYDCross(n, v0) < 0 && FXYDCross(n, v4) > 0) {
					FXY a = Outpoint[0] - Outpoint[3];
					float excess2 = FXYDDot(n, a) + dist3;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			if (value1 > 0) {
				// 垂線と線分の両点の外積
				if (FXYDCross(n, v1) < 0 && FXYDCross(n, v5) > 0) {
					FXY a = Outpoint[1] - Outpoint[0];
					//TRACE(_T("a:(x %f, y %f) \n"), a.x, a.y);
					//TRACE(_T("p1:(x %f, y %f) \n"), Outpoint[1].x, Outpoint[1].y);
					//TRACE(_T("dot:%f\n"), FXYDDot(n, a));
					float excess2 = FXYDDot(n, a) + dist0;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			if (value2 > 0) {
				// 垂線と線分の両点の外積
				if (FXYDCross(n, v2) < 0 && FXYDCross(n, v6) > 0) {
					FXY a = Outpoint[2] - Outpoint[1];
					float excess2 = FXYDDot(n, a) + dist1;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			if (value3 > 0) {
				// 垂線と線分の両点の外積
				if (FXYDCross(n, v3) < 0 && FXYDCross(n, v7) > 0) {
					FXY a = Outpoint[3] - Outpoint[2];
					float excess2 = FXYDDot(n, a) + dist2;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			
			point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
			point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
			point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
			point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).

			GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);

			//// 始点と円の中心との距離
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// 終点と円の中心との距離
			//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
			//	VectorNormalize(v2);
			//	Pos = seg.ep() + v2 * Radius;
			//	calcReflect(v2);
			//	Vel *= Restitution;
			//}

		}
	}

	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	void draw(HDC& hdc) {
		RenderRotRectangle(hdc, Pos, point, ag, Color);
	}
};

// 矩形のコンストラクタ
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// 幅
	h = height;		// 高さ

	ag = angle;	// 角度(45度回す).

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// 左上(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// 左下(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// 右下(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// 右上(x, y).
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 10;	// 0.1秒間隔で更新.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// 各種オブジェクトの初期配置.
// 「初期配置リセット」のボタンを押したらこの関数が呼ばれます.
// その後、UpdateObject()がunTimer == 0 で１回だけ呼ばれます.
// 各図形の初期位置をこちらで設定してください.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	// 線分の始点と終点（コップ）
	{
		float p[] = {
		g_sCupPoint[0].x, g_sCupPoint[0].y,
		g_sCupPoint[1].x, g_sCupPoint[1].y,
		g_sCupPoint[2].x, g_sCupPoint[2].y,
		g_sCupPoint[3].x, g_sCupPoint[3].y,
		};

		segments.clear();

		int numLine = sizeof(p) / 8 - 1;
		for (int i = 0; i < numLine; i++) {
			segments.emplace_back(
				p[i * 2 + 0],
				p[i * 2 + 1],
				p[i * 2 + 2],
				p[i * 2 + 3]
			);
		}
	}

	// 矩形の初期化
	{
		// rects.clear();

		for (int i = 0; i < 10; i++) {

			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float width = RandomRangeF(50.0f, 150.0f);
			float height = RandomRangeF(50.0f, 150.0f);
			float angle = RandomRangeF(0.0f, 360.0f);
			//float width = 100;
			//float height = 50;
			//float angle = 275;
			float restitution = 0.5f;

			rects.emplace_back(rgb, px, py, width, height, angle, restitution);
		}
	}

	/*
	// 円の初期化
	{
		balls.clear();

		for (int i = 0; i < 10; i++) {

			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float radius = RandomRangeF(20.0f, 50.0f);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float restitution = 0.5f;

			balls.emplace_back(rgb, radius, px, py, restitution);
		}
	}
	*/
}

void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset, HDC& hdc);


// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(&bReset, hdc);

}

// 位置更新
void UpdatePos(bool* PtrbReset, HDC& hdc) {
	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		FXY gravity(0, 0.2f);

		// 円の位置更新
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity);
				ball.move();
				for (BALL& other : balls) {
					ball.collisionBall(other);
				}
				for (SEGMENT& segment : segments) {
					ball.collisionSegment(segment);
				}
				ball.draw(hdc);
			}

		}

		// 矩形の位置更新
		{
			for (RECTANGLE& rect : rects) {
				rect.applyForce(gravity);
				rect.move();
				//for (BALL& other : balls) {
				//	rect.collisionBall(other);
				//}
				for (SEGMENT& segment : segments) {
					rect.collisionSegment(segment);
				}
				rect.draw(hdc);
			}

		}
	}

}
// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}

#endif

// 何かを試みようとした痕跡
#ifdef TEST00

float FXYDDot(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分の構造体

class SEGMENT {
	FXY Sp;
	FXY Ep;
	FXY N; // 垂線ベクトル
public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
};

SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	FXY v = Ep - Sp;
	N.x = -v.y;
	N.y = v.x;
	VectorNormalize(N);
}

SEGMENT segment1(g_sCupPoint[0].x, g_sCupPoint[0].y, g_sCupPoint[1].x, g_sCupPoint[1].y);
SEGMENT segment2(g_sCupPoint[1].x, g_sCupPoint[1].y, g_sCupPoint[2].x, g_sCupPoint[2].y);
SEGMENT segment3(g_sCupPoint[2].x, g_sCupPoint[2].y, g_sCupPoint[3].x, g_sCupPoint[3].y);

class BALL {

	FXY Pos, Vel;
	float Speed = 1.0f;
	float Radius = 0.1f;
	UINT rgb;
public:
	BALL(float px, float py, float vx, float vy, float sp, float r);

	const FXY& getPos() { return Pos; }
	const float& getRad() { return Radius; }
	const UINT& getRGB() { return rgb; }
	void setPos(const FXY& pos) { Pos = pos; }
	void setRad(const float& Rad) { Radius = Rad; }
	void setRgb(const UINT& RGB) { rgb = RGB; }
	void setVel(const FXY& vel) {
		Vel = vel;
		Vel *= Speed;
	}
	void manual() {
		Vel.x = Vel.y = 0;
		if (GetAsyncKeyState('D') & 0x01) Vel.x = 1;
		if (GetAsyncKeyState('A') & 0x01) Vel.x = -1;
		if (GetAsyncKeyState('W') & 0x01) Vel.y = -1;
		if (GetAsyncKeyState('S') & 0x01) Vel.y = 1;
		Vel *= Speed;
	}
	void move() {
		Pos += Vel;
	}
	void collision(SEGMENT& seg) {
		FXY n = seg.n();
		FXY v = Pos - seg.sp();
		float dist = FXYDDot(n, v);
		if (abs(dist) < Radius) {
			float excess = Radius - abs(dist);
			Pos += n * excess;
			Vel.x = FXYDDot(n, -Vel) * 2 * n.x + Vel.x;
			Vel.y = FXYDDot(n, -Vel) * 2 * n.y + Vel.y;
		}
	}
};
BALL::BALL(float px, float py, float vx, float vy, float sp, float r) {
	rgb = RGB(144, 238, 144);
	Pos.x = px; Pos.y = py; Vel.x = vx; Vel.y = vy;
	Speed = sp;
	Radius = r;
}

BALL ball(100, 200, 0, 0, 0.03f, 0.3f);	// 円.

template
<
	typename TYPE,
	std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE])
{
	return SIZE;
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 100;	// 0.1秒間隔で更新.
}

void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	FXY xy;
	xy.x = RandomRangeF(200.0f, 500.0f);
	xy.y = RandomRangeF(100.0f, 200.0f);
	ball.setPos(xy);
	float r;
	r = RandomRangeF(20.0f, 50.0f);
	ball.setRad(r);
	UINT rgb = RGB(144, 238, 144);
	ball.setRgb(rgb);
}
void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset);

// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(PtrbReset);

	// 描画
	Draw(hdc);
}

// 描画する.
void Draw(HDC& hdc) {
	// 座標(x, y)中心に半径rの円を描画する.
	RenderCircle(hdc, ball.getPos().x, ball.getPos().y, ball.getRad(), ball.getRGB());
}

// 位置更新
void UpdatePos(bool* PtrbReset) {
	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		// 円の位置更新
		{
			ball.manual();
			ball.move();
			ball.collision(segment2);

		}
	}

}
// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}


#endif

// 円同士のコリジョン解決　※発表に使用
#ifdef VECTOR_BALL

// プロトタイプ宣言
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分のクラス
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // 垂線ベクトル
public:
	// コンストラクタ
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// コンストラクタ
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	FXY v = VectorSub(Sp, Ep);
	N.x = -v.y;
	N.y = v.x;
	VectorNormalize(N);
}

// 円のクラス
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
public:
	// コンストラクタ
	BALL(UINT color, float r, float px, float py, float restitution);

	// 加速度
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 移動
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	void collision(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// 線分と円の中心との距離（内積）
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();
			// 垂線と線分の両点の外積
			if (FXYDCross(n,v1) < 0 && FXYDCross(n, v2) > 0) {
				float excess = Radius - abs(dist);
				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;
				calcReflect(n);
				Vel *= Restitution;
			}
			// 始点と円の中心との距離
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}
			// 終点と円の中心との距離
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// 円同士の衝突判定と衝突応答
	void collisionBall(BALL& other) {
		if (this != &other) {
			FXY v = Pos - other.Pos;
			float dist = Radius + other.Radius;
			if (v.x * v.x + v.y * v.y < dist * dist) {
				float excess = dist - VectorLength(v);
				VectorNormalize(v);
				Pos += v * excess * 0.6f;
				other.Pos -= v * excess * 0.6f;
				calcReflect(v);
				Vel *= Restitution;
			}
		}
	}

	// 反射ベクトルを求める
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// 描画
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// コンストラクタ
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 10;	// 0.1秒間隔で更新.
}

vector<SEGMENT> segments;
vector<BALL> balls;

// 各種オブジェクトの初期配置.
// 「初期配置リセット」のボタンを押したらこの関数が呼ばれます.
// その後、UpdateObject()がunTimer == 0 で１回だけ呼ばれます.
// 各図形の初期位置をこちらで設定してください.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	{    
		float p[] = {
		g_sCupPoint[0].x, g_sCupPoint[0].y,
		g_sCupPoint[1].x, g_sCupPoint[1].y,
		g_sCupPoint[2].x, g_sCupPoint[2].y,
		g_sCupPoint[3].x, g_sCupPoint[3].y,
		};

		segments.clear();

		int numLine = sizeof(p) / 8 - 1;
		for (int i = 0; i < numLine; i++) {
			segments.emplace_back(
				p[i * 2 + 0],
				p[i * 2 + 1],
				p[i * 2 + 2],
				p[i * 2 + 3]
			);
		}
	}

	{
		// これをオンにするとResetObjectが呼ばれた時に配列の中身を消す
		// balls.clear();

		for (int i = 0; i < 10; i++) {
			
			UINT rgb = RGB(RandomRangeF(0.0f, 255.0f), RandomRangeF(0.0f, 255.0f), 255);
			float radius = RandomRangeF(20.0f, 50.0f);
			float px = RandomRangeF(200.0f, 500.0f);
			float py = RandomRangeF(100.0f, 200.0f);
			float restitution = 0.5f;
				
			balls.emplace_back(rgb, radius, px, py, restitution);
		}
	}

}

void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset, HDC& hdc);

// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(&bReset, hdc);

}

// 位置更新
void UpdatePos(bool* PtrbReset, HDC& hdc) {
	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		FXY gravity(0, 0.2f);

		// 円の位置更新
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity); // 加速度
				ball.move(); // 移動
				for (BALL& other : balls) {
					ball.collisionBall(other); // 円との衝突判定
				}
				for (SEGMENT& segment : segments) {
					ball.collision(segment); // 線分との衝突判定
				}
				ball.draw(hdc); // 描画
			}

		}
	}

}
// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}

#endif

// 配布されたプログラムから最低限関数化した状態
#ifdef BASE
float FXYDDot(FXY sLhs, FXY sRhs);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // コップの四隅の座標.

// 線分の構造体
struct SEGMENT {
public:
	FXY Sp;
	FXY Ep;
	FXY N; // 垂線ベクトル

	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
};

SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	FXY v = Ep - Sp;
	N.x = -v.y;
	N.y = v.x;
	VectorNormalize(N);
}

SEGMENT segment1(g_sCupPoint[0].x, g_sCupPoint[0].y, g_sCupPoint[1].x, g_sCupPoint[1].y);
SEGMENT segment2(g_sCupPoint[1].x, g_sCupPoint[1].y, g_sCupPoint[2].x, g_sCupPoint[2].y);
SEGMENT segment3(g_sCupPoint[2].x, g_sCupPoint[2].y, g_sCupPoint[3].x, g_sCupPoint[3].y);

// 例：円の構造体.
struct S_CIRCLE {
public:
	float r;
	UINT rgb;

	// 座標(x, y)に四角形を描画する 座標中心は四角形の重心に設定
	FXY center;

	S_CIRCLE();
};

// コンストラクタ
S_CIRCLE::S_CIRCLE() {
	center.x = 100;
	center.y = 100;
	r = 10;
	rgb = RGB(144, 238, 144);
}

S_CIRCLE g_sCircle[5];	// 円.

// 例：四角形の構造体.
struct S_SQUARE {
public:
	float w;		// 幅
	float h;		// 高さ

	float angle;			

	UINT rgb;
	// 座標(x, y)に四角形を描画する 座標中心は四角形の重心に設定
	FXY center;

	FXY point[4];	// rad = 0 時の各頂点の座標

	S_SQUARE();
};

// コンストラクタ
S_SQUARE::S_SQUARE() {
	w = 50;			// 幅
	h = 100;		// 高さ

	angle = 45.0f;	// 角度(45度回す).

	rgb = RGB(144, 238, 144);

	center.x = 400;
	center.y = 200;

	point[0] = { center.x - w / 2.0f, center.y - h / 2.0f };	// 左上(x, y).
	point[1] = { center.x - w / 2.0f, center.y + h / 2.0f };	// 左下(x, y).
	point[2] = { center.x + w / 2.0f, center.y + h / 2.0f };	// 右下(x, y).
	point[3] = { center.x + w / 2.0f, center.y - h / 2.0f };	// 右上(x, y).
}

S_SQUARE g_sSquare;	// 四角形.

// 例：二等辺三角形の構造体.
struct S_TRIANGLE {
public:
	float w;		// 底辺
	float h;		// 高さ

	// 図形の回転角を指定します.
	// 角度は必ず0.0～360.0に収めてください(範囲を外れると計算誤差が大きくなるため).
	// FIT_ANGLE()のマクロを通せば範囲に収める値を返します.
	// 例) FIT_ANGLE(362.0f) → 2.0f を戻します.
	//     FIT_ANGLE(-10.0f) → 350.0f を戻します.
	float angle;

	UINT rgb;
	// 座標(x, y)に二等辺三角形を描画する 座標中心は二等辺三角形の重心に設定
	FXY center;

	FXY point[3];	// rad = 0 時の各頂点の座標

	S_TRIANGLE();
};

// コンストラクタ
S_TRIANGLE::S_TRIANGLE() {
	w = 100.0f;			// 底辺
	h = 70.0f;			// 高さ

	angle = 30.0f;			// 角度(30度回す).

	rgb = RGB(144, 238, 144);

	center.x = 400;
	center.y = 200;

	point[0] = { center.x + w / 2.0f, center.y - h / 3.0f };
	point[1] = { center.x - w / 2.0f, center.y - h / 3.0f };
	point[2] = { center.x,	          center.y + (2.0f * h) / 3.0f };
}

S_TRIANGLE g_sTriangle;	// 二等辺三角形.

template
<
	typename TYPE,
	std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE])
{
	return SIZE;
}

// 乱数シードを取得する.
// シード値を固定化すると、毎回同じ乱数が発生します。
// つまり毎回同じ配置状態からスタートします.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// もしもシード値を固定化したい場合は↓をコメントアウトしてください.
	unSeed = static_cast<UINT>(time(NULL));	// こちらが有効だと毎回異なる乱数が発生します.

	return unSeed;
}


// 更新頻度を取得する.
// コマ送りで動作させることができます.
UINT GetUpdateMsec()
{
	// 更新時間(1000で1秒、ゆっくり更新したければ数字を大きくする).
	// 実時間通りに計算する必要はありません.
	return 10;	// 0.1秒間隔で更新.
}


// 各種オブジェクトの初期配置.
// 「初期配置リセット」のボタンを押したらこの関数が呼ばれます.
// その後、UpdateObject()がunTimer == 0 で１回だけ呼ばれます.
// 各図形の初期位置をこちらで設定してください.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printfの代わりに利用できます。（出力ウインドウに表示されます）.

	{
		for (int i = 0; i < array_length(g_sCircle); i++) {
			// 円の初期位置をランダムで設定.
			while (1) {
				g_sCircle[i].center.x = RandomRangeF(200.0f, 500.0f);	// 200～500 の間のランダムな数字.
				g_sCircle[i].center.y = RandomRangeF(100.0f, 200.0f);
				g_sCircle[i].r = RandomRangeF(20.0f, 50.0f);
				g_sCircle[i].rgb = RGB(144, 238, 144);
				if (i == 0) {
					break;
				}
				float dis;
				float Rad;
				bool button = true;

				for (int j = 0; j < i; j++) {
					dis = CalcDistance(g_sCircle[i].center, g_sCircle[j].center);
					Rad = g_sCircle[i].r + g_sCircle[j].r;

					//TRACE(_T("%d to %d Dis: %f \n"),i,j, dis);
					//TRACE(_T("Sum %d of %d Radius: %f \n"), i, j, Rad);

					if (CalcDistance(g_sCircle[i].center, g_sCircle[j].center) < Rad) {

						//TRACE(_T("break \n"));
						button = false;

					}
				}

				if (button) {
					break;
				}
			}

		}
	}

	{
		// 四角形の初期位置をランダムで設定.
		g_sSquare.w = RandomRangeF(50.0f, 200.0f);
		g_sSquare.h = RandomRangeF(50.0f, 200.0f);
		g_sSquare.angle = RandomRangeF(0.0f, 360.0f);

		g_sSquare.center.x = RandomRangeF(200.0f, 500.0f);	// 200～500 の間のランダムな数字.
		g_sSquare.center.y = RandomRangeF(100.0f, 200.0f);
		g_sSquare.rgb = RGB(30, 144, 255);	// 色.
	}

	{
		// 二等辺三角形の初期位置をランダムで設定.
		g_sTriangle.w = RandomRangeF(50.0f, 200.0f);
		g_sTriangle.h = RandomRangeF(50.0f, 200.0f);
		g_sTriangle.angle = RandomRangeF(0.0f, 360.0f);

		g_sTriangle.center.x = RandomRangeF(200.0f, 500.0f);	// 200～500 の間のランダムな数字.
		g_sTriangle.center.y = RandomRangeF(100.0f, 200.0f);
		g_sTriangle.rgb = RGB(250, 140, 0);	// 色.
	}

}

void DetectColiderSphere(SEGMENT& seg);
void CalcPoint(int x);
void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset);

// 各種オブジェクトの更新.
// 「コリジョン解決」のボタンを押すと、一定間隔で本関数が呼ばれます(もう一度押すと止まります).
// 呼ばれるタイミングを変えたいときは GetUpdateMsec() を変更してください(初期状態は0.1秒間隔です).
// 別に時間に合わせて処理せず、一瞬で落下して解決状態としても問題ありません.
// 物理法則に則って落下する必要もありません(それっぽい計算で十分です).
// 引数：unTimer 呼ばれる度にカウントアップします(0は初期状態).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// 位置更新
	UpdatePos(PtrbReset);

	// 描画
	Draw(hdc);
}

// コップのコリジョン判定 (円)
void DetectColiderSphere(SEGMENT& seg) {
	const FXY& n = seg.n();

	for (int i = 0; i < array_length(g_sCircle); i++) {
		FXY& v = g_sCircle[i].center - seg.sp();

		float dist = FXYDDot(n, v);
		float Radius = g_sCircle[i].r;

		if (abs(dist) < Radius) {
			float excess = Radius - abs(dist);
			g_sCircle[i].center.y += 0.0f;
		}
		else {
			g_sCircle[i].center.y += 2.0f;	// 落下.
		}
		//if (g_sCircle[i].center.y + g_sCircle[i].r >= g_sCupPoint[2].y) {
		//	g_sCircle[i].center.y += 0.0f;
		//}
		//else {
		//	g_sCircle[i].center.y += 2.0f;	// 落下.
		//}
	}
}

// 円の支配領域
void SphereBoundary() {
	
}

// 頂点位置計算（毎フレーム）
void CalcPoint(int x) {
	// 頂点が三つの時
	if (x == 3) {
		g_sTriangle.point[0] = { g_sTriangle.center.x + g_sTriangle.w / 2.0f, g_sTriangle.center.y - g_sTriangle.h / 3.0f };
		g_sTriangle.point[1] = { g_sTriangle.center.x - g_sTriangle.w / 2.0f, g_sTriangle.center.y - g_sTriangle.h / 3.0f };
		g_sTriangle.point[2] = { g_sTriangle.center.x,	          g_sTriangle.center.y + (2.0f * g_sTriangle.h) / 3.0f };

	}
	// 頂点が四つの時
	else if (x == 4) {
		g_sSquare.point[0] = { g_sSquare.center.x - g_sSquare.w / 2.0f, g_sSquare.center.y - g_sSquare.h / 2.0f };	// 左上(x, y).
		g_sSquare.point[1] = { g_sSquare.center.x - g_sSquare.w / 2.0f, g_sSquare.center.y + g_sSquare.h / 2.0f };	// 左下(x, y).
		g_sSquare.point[2] = { g_sSquare.center.x + g_sSquare.w / 2.0f, g_sSquare.center.y + g_sSquare.h / 2.0f };	// 右下(x, y).
		g_sSquare.point[3] = { g_sSquare.center.x + g_sSquare.w / 2.0f, g_sSquare.center.y - g_sSquare.h / 2.0f };	// 右上(x, y).
	}
}

// 描画する.
void Draw(HDC& hdc) {
	// 三角形を描画する.
	RenderRotTriangle(hdc, g_sTriangle.center, g_sTriangle.point, g_sTriangle.angle, g_sTriangle.rgb);

	// 四角形を描画する.
	RenderRotRectangle(hdc, g_sSquare.center, g_sSquare.point, g_sSquare.angle, g_sSquare.rgb);

	// 座標(x, y)中心に半径rの円を描画する.

	for (int i = 0; i < array_length(g_sCircle); i++) {
		RenderCircle(hdc, g_sCircle[i].center.x, g_sCircle[i].center.y, g_sCircle[i].r, g_sCircle[i].rgb);
	}
}

// 位置更新
void UpdatePos(bool* PtrbReset) {
	// リセットじゃなければ座標を更新する.
	if (&PtrbReset) {

		// 二等辺三角形の位置更新
		{
			g_sTriangle.angle = FIT_ANGLE(g_sTriangle.angle);		// 必須：0.0f～360.0fの角度に収める.

			CalcPoint(3);
			g_sTriangle.center.y += 2.0f;	// 落下.
		}
	}
	if (&PtrbReset) {

		// 四角形の位置更新
		{
			g_sSquare.angle = FIT_ANGLE(g_sSquare.angle);		// 必須：0.0f～360.0fの角度に収める.

			CalcPoint(4);
			g_sSquare.center.y += 2.0f;	// 落下.
		}
	}
	if (&PtrbReset) {

		// 円の位置更新
		{
			//DetectColiderSphere(segment1);
			DetectColiderSphere(segment2);
			//DetectColiderSphere(segment3);
		}
	}

}
// コップの描画.
// こちらは変更の必要はないはず.
void RenderCup(HDC& hdc)
{
	// コップの描画
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// 描画
		HPEN hPen = CreatePen(PS_SOLID, 5, RGB(0, 0, 0));
		HGDIOBJ hOldObj = SelectObject(hdc, hPen);
		for (int i = 1; i < pointCount; i++) {
			MoveToEx(hdc, (int)g_sCupPoint[i - 1].x, (int)g_sCupPoint[i - 1].y, NULL);
			LineTo(hdc, (int)g_sCupPoint[i].x, (int)g_sCupPoint[i].y);
		}
		SelectObject(hdc, hOldObj);
		DeleteObject(hPen);
	}
}

#endif

/* ▲▲▲  ▲▲▲ ここまでを編集してください ▲▲▲  ▲▲▲ */

// 乱数取得.
int Random(int nMax)
{
	return rand() % (nMax + 1);
}

// 乱数取得.
int RandomRange(int nMin, int nMax)
{
	if (nMax >= nMin) {
		TRACE(_T("最大値が最小値より大きくなるように設定してください\n"));
		ASSERT(false);
		return nMin;
	}
	UINT nRnd = static_cast<UINT>(nMax - nMin + 1);
	return (nMin + rand() % nRnd);
}

// 乱数取得.
float RandomF(float fMax)
{
	return static_cast<float>(rand() % static_cast<UINT>(fMax + 1));
}

// 乱数取得.
float RandomRangeF(float fMin, float fMax)
{
	if (fMin >= fMax) {
		TRACE(_T("最大値が最小値より大きくなるように設定してください\n"));
		ASSERT(false);
		return fMin;
	}
	UINT nRnd = static_cast<UINT>(fMax - fMin + 1);
	return static_cast<float>(fMin + rand() % nRnd);
}


// ２地点間の距離.
float CalcDistance(FXY sLhs, FXY sRhs)
{
	FXY sSub = sRhs - sLhs;
	return sqrtf(sSub.x * sSub.x + sSub.y * sSub.y);
}

// ベクトルのサイズ.
float VectorLength(FXY sXy)
{
	return sqrtf(sXy.x * sXy.x + sXy.y * sXy.y);
}

// ベクトルの差
FXY VectorSub(FXY sFrom, FXY sTo)
{
	return sTo - sFrom;
}

// ベクトル正規化.
void VectorNormalize(FXY& sXy)
{
	float fLength = sXy.x * sXy.x + sXy.y * sXy.y;
	ASSERT(fLength);
	if (fLength > 0) {
		fLength = 1.0f / sqrtf(fLength);
		sXy.x *= fLength;
		sXy.y *= fLength;
	}
}

// ベクトルの内積.
float FXYDDot(FXY sLhs, FXY sRhs)
{
	return (sLhs.x * sRhs.x) + (sLhs.y * sRhs.y);
}

// ベクトルの外積.
float FXYDCross(FXY sLhs, FXY sRhs)
{
	return (sLhs.x * sRhs.y) - (sLhs.y * sRhs.x);
}


// 回転後の座標取得.
// pOutput	回転計算後の各頂点の座標値配列.
// center	回転の中心座標.
// pPos		回転計算前の各頂点の座標値配列.
// nNum		座標値配列の個数(三角形なら3, 四角形なら4を渡す).
// fAngle	回転角度.
void GetRotPos(FXY* pOutput, FXY center, FXY* pPos, int nNum, float fAngle)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("角度が0.0～360.0に収まっていません[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()を通して戻された値を次回以降も利用する必要があります.
	}
	float rad = ANGLE(fAngle);	// ラジアン.
	float cosValue = cosf(rad);
	float sinValue = sinf(rad);
	for (int i = 0; i < nNum; i++) {
		// 回転変換
		pOutput[i].x = center.x + (pPos[i].x - center.x) * cosValue - (pPos[i].y - center.y) * sinValue;
		pOutput[i].y = center.y + (pPos[i].x - center.x) * sinValue + (pPos[i].y - center.y) * cosValue;
	}
}

// 多角形の描画.
void RenderPolygon(HDC hdc, POINT* pPos, int nNum, UINT rgb)
{
	HBRUSH hBrush = CreateSolidBrush(rgb);
	HGDIOBJ hOldObj = SelectObject(hdc, hBrush);	// 塗りつぶし色設定
	SetPolyFillMode(hdc, WINDING);
	Polygon(hdc, pPos, nNum);
	SelectObject(hdc, hOldObj);	// 元の色に復帰
	DeleteObject(hBrush);
}

// 三角形の描画.
void RenderTriangle(HDC hdc, POINT* pPos, UINT rgb)
{
	RenderPolygon(hdc, pPos, 3, rgb);
}
void RenderTriangle(HDC hdc, FXY* pPos, UINT rgb)
{
	POINT sXy[3];
	for (int i = 0; i < 3; i++) {
		sXy[i].x = static_cast<int>(pPos[i].x);
		sXy[i].y = static_cast<int>(pPos[i].y);
	}
	RenderPolygon(hdc, sXy, 3, rgb);
}

// 回転した三角形の描画.
// center	回転の中心.
// pPos		各頂点座標.
// fAngle	回転角(0.0～360.0).
// rgb		三角形の色.
void RenderRotTriangle(HDC hdc, FXY& center, FXY* pPos, float fAngle, UINT rgb)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("角度が0.0～360.0に収まっていません[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()を通して戻された値を次回以降も利用する必要があります.
	}
	
	// 回転後の座標を取得する.
	FXY radPos[3];
	GetRotPos(radPos, center, pPos, COUNTOF(radPos), fAngle);
	
	POINT drawPoint[3];		// 角度を考慮した描画時の座標(描画APIの都合で整数になっています)
	for (int i = 0; i < 3; i++) {
		drawPoint[i] = { static_cast<LONG>(radPos[i].x), static_cast<LONG>(radPos[i].y) };
	}

	// 三角形の描画
	RenderTriangle(hdc, drawPoint, rgb);
}


// 四角形の描画.
void RenderRectangle(HDC hdc, POINT* pPos, UINT rgb)
{
	RenderPolygon(hdc, pPos, 4, rgb);
}
void RenderRectangle(HDC hdc, FXY* pPos, UINT rgb)
{
	POINT sXy[4];
	for (int i = 0; i < 4; i++) {
		sXy[i].x = static_cast<int>(pPos[i].x);
		sXy[i].y = static_cast<int>(pPos[i].y);
	}
	RenderPolygon(hdc, sXy, 4, rgb);
}

// 回転した四角形の描画.
// center	回転の中心.
// pPos		各頂点座標.
// fAngle	回転角(0.0～360.0).
// rgb		四角形の色.
void RenderRotRectangle(HDC hdc, FXY& center, FXY* pPos, float fAngle, UINT rgb)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("角度が0.0～360.0に収まっていません[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()を通して戻された値を次回以降も利用する必要があります.
	}
	
	// 回転後の座標を取得する.
	FXY radPos[4];	// 配列数に注意.
	GetRotPos(radPos, center, pPos, COUNTOF(radPos), fAngle);

	POINT drawPoint[4];		// 角度を考慮した描画時の座標
	for (int i = 0; i < 4; i++) {
		drawPoint[i] = { static_cast<LONG>(radPos[i].x), static_cast<LONG>(radPos[i].y) };
	}

	// 四角形の描画.
	RenderRectangle(hdc, drawPoint, rgb);
}


// 円の描画.
// 座標(x, y)に半径rの円を描画する.
void RenderCircle(HDC hdc, float x, float y, float r, UINT rgb)
{
	int nX = static_cast<int>(x);
	int nY = static_cast<int>(y);
	int nR = static_cast<int>(r);

	// 描画
	HBRUSH hBrush = CreateSolidBrush(rgb);
	HGDIOBJ hOldObj = SelectObject(hdc, hBrush);	// 塗りつぶし色設定
	Ellipse(hdc, nX - nR, nY - nR, nX + nR, nY + nR);
	SelectObject(hdc, hOldObj);	// 元の色に復帰
	DeleteObject(hBrush);
}
