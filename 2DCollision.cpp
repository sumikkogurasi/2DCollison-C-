#include "stdafx.h"
#include "GUIObject.h"
#include <time.h>

// �ǉ������W�����C�u����
#include <array>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

#define VECTOR_ADD_RECT_ROT_COMBINE_BALL

// ��`�ɉ�]��ǉ��o���Ȃ��������ߎO�p�`�͒��߂܂����B

// ��`�Ɖ~�̃R���W���������i�������j
#ifdef VECTOR_ADD_RECT_ROT_COMBINE_BALL

// �v���g�^�C�v�錾
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̃N���X
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // �����x�N�g��
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// �����̃R���X�g���N�^
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	V = VectorSub(Sp, Ep);

	// �@���x�N�g��
	N.x = -V.y;
	N.y = V.x;

	// ���ꂼ�ꐳ�K��
	VectorNormalize(N);
	VectorNormalize(V);
}

// �~�̃N���X
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
	float rate = 0.6f; // �Փˉ����Ɏg���W��
public:
	BALL(UINT color, float r, float px, float py, float restitution);

	const FXY& pos() { return Pos; }
	const float& radius() { return Radius; }

	// �����x
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	// �����Ƃ̏Փ˔���A����
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// �����Ɖ~�̒��S�Ƃ̋����i���ρj
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();

			// �����Ɛ����̗��_�̊O��
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {

				// �߂荞�݂����������v�Z
				float excess = Radius - abs(dist);

				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;

				calcReflect(n);
				Vel *= Restitution;
			}

			// �n�_�Ɖ~�̒��S�Ƃ̋���
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}

			// �I�_�Ɖ~�̒��S�Ƃ̋���
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// �~�Ƃ̏Փ˔���A����
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

	// ���˃x�N�g�������߂�
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// �`��
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// �~�̃R���X�g���N�^
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// ��`�̃N���X
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// ��
	float h = 0.0f;		// ����
	float ag = 0.0f;	// �p�x
	UINT Color;

	FXY point[4];	// rad = 0 ���̊e���_�̍��W
	FXY Outpoint[4];	// rad = ag/pi ���̊e���_�̍��W

	float Restitution = 0.0f; // �����W��

	float state = false;

	float dir = 0.0f; // �O�ς̕����m�F�p

	FXY Axis; // ���̂ɏՓ˂������ɐڐG�����_�̈ʒu��ۊ�

	FXY N[4]; // ��`�̕ӂ̖@���x�N�g��

	int num = 5;

	bool situation = false;

	FXY mv[4]; // �������g�̕ӂ̃x�N�g��

	bool StaticRect = false; // �R�b�v�ƐڐG����_�̐��ɉ����ĕ��̂��Œ肷��t���O

	bool StaticRect2 = false; // ��`�ƐڐG����_�̐��ɉ����ĕ��̂��Œ肷��t���O

	int count2 = 0; // �R�b�v�Ƃ̐ڐG�_�̐�

	int count3 = 0; // ��`�Ƃ̐ڐG�_�̐�

	FXY LOG; // �����Ƃ̏Փ˔���I�����̌v�Z���ʂ�ۊ�

public:

	// �R���X�g���N�^
	RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution);

	const FXY& OUTPOINT(int i) { return Outpoint[i]; }

	// �����x
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// �ړ�
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;

		ag = FIT_ANGLE(ag);

		// �l�p�`�̌`�󂩂��]���l���������_�̈ʒu�����߂�
		CalcPoint();

		count2 = 0;

		count3 = 0;

		StaticRect = false;

		StaticRect2 = false;

	}

	// �����Ƃ̏Փ˔���A����
	void collisionSegment(SEGMENT& seg) {

		const FXY& n = seg.n();
		const FXY& v = seg.v();

		// �����̎n�_�����`��4���_�ւ̃x�N�g��
		FXY v0 = Outpoint[0] - seg.sp();
		FXY v1 = Outpoint[1] - seg.sp();
		FXY v2 = Outpoint[2] - seg.sp();
		FXY v3 = Outpoint[3] - seg.sp();

		// �����Ƌ�`�̒��_�܂ł̋���
		float dist0 = FXYDDot(n, v0);
		float dist1 = FXYDDot(n, v1);
		float dist2 = FXYDDot(n, v2);
		float dist3 = FXYDDot(n, v3);

		// ��`�̒��_�������̍��E�ǂ���ɂ��邩�����Ŕ��f����
		float value0 = FXYDCross(v, v0);
		float value1 = FXYDCross(v, v1);
		float value2 = FXYDCross(v, v2);
		float value3 = FXYDCross(v, v3);

		// ��`�̒��_�����ׂĐ����ɑ΂��ē��������ɂ������ꍇ
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {

			state = true;

			// �����̏I�_�����`��4���_�ւ̃x�N�g��
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();

			// ��`�̒��_�����`�̏d�S�܂ł̃x�N�g��
			FXY centerV0 = Pos - Outpoint[0];
			FXY centerV1 = Pos - Outpoint[1];
			FXY centerV2 = Pos - Outpoint[2];
			FXY centerV3 = Pos - Outpoint[3];

			// �����̖@���x�N�g���Ƌ�`�̒��_�����`�̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��p
			float valueN0 = FXYDCross(n, centerV0);
			float valueN1 = FXYDCross(n, centerV1);
			float valueN2 = FXYDCross(n, centerV2);
			float valueN3 = FXYDCross(n, centerV3);

			// �����̃x�N�g���Ƌ�`�̒��_�����`�̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��p
			float valueC0 = FXYDCross(v, centerV0);
			float valueC1 = FXYDCross(v, centerV1);
			float valueC2 = FXYDCross(v, centerV2);
			float valueC3 = FXYDCross(v, centerV3);

			if (value0 > 0) {

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

			// �����ƐڐG�������_�̐�����ȏ�̏ꍇ���̂��Œ肷��
			if (count3 >= 2) {
				StaticRect = true;
			}

			// �Œ肳��ĂȂ��������]������
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
			//// �n�_�Ɖ~�̒��S�Ƃ̋���
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// �I�_�Ɖ~�̒��S�Ƃ̋���
			//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
			//	VectorNormalize(v2);
			//	Pos = seg.ep() + v2 * Radius;
			//	calcReflect(v2);
			//	Vel *= Restitution;
			//}

		}
	}

	// ��`�Ƃ̏Փ˔���A����
	void collisionRectangle(RECTANGLE& other) {
		if (this != &other) {

			// �ڐG������`�̕ӂ̃x�N�g��
			FXY ov[4];
			ov[0] = other.Outpoint[1] - other.Outpoint[0];
			ov[1] = other.Outpoint[2] - other.Outpoint[1];
			ov[2] = other.Outpoint[3] - other.Outpoint[2];
			ov[3] = other.Outpoint[0] - other.Outpoint[3];

			// [����][����] ���_�ւ̃x�N�g��
			FXY inside[4][4];
			for (int i = 0; i < 4; i++) {
				inside[0][i] = Outpoint[i] - other.Outpoint[0];
				inside[1][i] = Outpoint[i] - other.Outpoint[1];
				inside[2][i] = Outpoint[i] - other.Outpoint[2];
				inside[3][i] = Outpoint[i] - other.Outpoint[3];
			}


			// [����][����] �O��
			float value[4][4];
			for (int i = 0; i < 4; i++) {
				value[0][i] = FXYDCross(ov[0], inside[0][i]);
				value[1][i] = FXYDCross(ov[1], inside[1][i]);
				value[2][i] = FXYDCross(ov[2], inside[2][i]);
				value[3][i] = FXYDCross(ov[3], inside[3][i]);
			}

			// ������4�̒��_���玩�����g�̏d�S�ւ̃x�N�g��
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

			// �����̒��_4�ɑ΂��ďՓ˂��Ă��邩���肷��
			for (int i = 0; i < 4; i++) {
				if (value[0][i] <= 0 && value[1][i] <= 0 && value[2][i] <= 0 && value[3][i] <= 0) {

					situation = true;

					// �����ƒ��_�Ƃ̋������ŏ��̐����̔ԍ����L�^���邽�߂ɒu�����i�v�C���j
					int min = 0;

					for (int j = 0; j < 4; j++) {
						// �����̐����Ƒ���̂ǂ̐����ƌ����������𔻒�
						if (CrossSegment(ov[j], -centerV[i], other.Outpoint[j], Pos)) {

							// ���̏ꍇ���ꂼ��ڐG�_���J�E���g����
							count2++;
							other.count2++;
							min = j;
						}
					}

					// ����̐���4�Ƒ���̋�`�ɐN�����������̒��_�Ƃ̋���
					float dist[4];

					// ���̌v�Z
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

					// ��`�ƐڐG���A��]����悤�ȏꍇ�ɕK�v�ȐڐG��������̐����̖@���x�N�g���Ǝ����̒��_���玩���̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��̂ɗp����
					float valueN[4];

					for (int i = 0; i < 4; i++) {
						valueN[i] = FXYDCross(-other.N[min], centerV[i]);
					}

					num = i;

					dir = valueN[i];

					// 
					RotPosCount(-ov[min], mv[i], mv[(i + 3) % 4]);

					// �����x�̋t�x�N�g���i�v���P�j
					FXY H;
					H.x = 0;
					H.y = -1;
					dir = FXYDCross(H, -centerV[i]);

					//other.Color = RGB(255, 255, 0);
					//Color = RGB(255, 255, 0);

					// ��̋�`���r���ĐG��Ă��钸�_�����ꂼ��2�ȏ�̏ꍇ
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

					//// �n�_�Ɖ~�̒��S�Ƃ̋���
					//
					//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
					//	VectorNormalize(v1);
					//	Pos = seg.sp() + v1 * Radius;
					//	calcReflect(v1);
					//	Vel *= Restitution;
					//}
					//// �I�_�Ɖ~�̒��S�Ƃ̋���
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

	// �~�Ƃ̏Փ˔���A����
	void collisionBall(BALL& ball) {
		const FXY& p = ball.pos();
		const float& r = ball.radius();

		// [����][����] ���_�ւ̃x�N�g��
		FXY inside[4];
		for (int i = 0; i < 4; i++) {

		}
	}

	// �d�S���ӂ̊O�ɏo�Ă��邩
	bool CenterRot(RECTANGLE& other, int min, int i) {

		// ���̓��m���Œ肵�Ă����ꍇ
		if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {
			TRACE(_T("CenterRot ME Sum: %d\n"), count3 + count2);
			TRACE(_T("CenterRot OTHER Sum: %d\n"), other.count2 + other.count3);
			return false;
		}

		// ����������̏�ɏd�Ȃ�ꍇ�Ŏ����̏d�S������̐����̗��[�̈ʒu����͂ݏo���Ă����ꍇ
		if (count2 + count3 < 2) {
			if (Pos.x > other.Outpoint[min].x || Pos.x < other.Outpoint[(min + 1) % 4].x) {
				if (Outpoint[(i + 1) % 4].y - other.Outpoint[min].y < 0 || Outpoint[(i + 3) % 4].y - other.Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// ���肪�����̏�ɏd�Ȃ�ꍇ�ő���̏d�S�������̐����̗��[�̈ʒu����͂ݏo���Ă����ꍇ
		if (other.count2 + other.count3 < 2) {
			if (other.Pos.x > Outpoint[min].x || other.Pos.x < Outpoint[(min + 1) % 4].x) {
				if (other.Outpoint[(i + 1) % 4].y - Outpoint[min].y < 0 || other.Outpoint[(i + 3) % 4].y - Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// �s���S
		return true;
	}

	// �Q�̐�����������Ă��邩����
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

		// 0 <= t <=1�̏ꍇ
		if (t1 < 0 || t1 > 1 || t2 < 0 || t2 > 1) {
			return false;
		}

		return true;
	}

	// ���˃x�N�g�����v�Z
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// �z����ōŏ��l������ԍ���Ԃ�
	int minValue(float nums[], int n) {
		float min_value; // �ő�l
		int i;
		int j = 0;
		// nums[0]���ő�l�Ɖ��肷��
		min_value = nums[0];

		for (i = 0; i < n; i++) {
			if (nums[i] < min_value) {
				/* �ő�l����nums[i]�̕����傫����΍ő�l���X�V */
				min_value = nums[i];
				j = i;
			}
		}

		return j;
	}

	// ��`�ƐڐG���ĉ�]�����㑊��̐����ɐڋ߂����ꍇ�ڐG�_��������
	// V�@�@�ڐG��������̃x�N�g��
	// V1�@ �ڐG�������_�̍����̃x�N�g��
	// V2�@ �ڐG�������_�̉E���̃x�N�g��

	void RotPosCount(FXY V, FXY V1, FXY V2)
	{
		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// �����̏d�S������̖@���̍����ɂ������ꍇ
		if (dir > 0) {

			// ����̐����Ǝ����̐����̃x�N�g�������s�ɋ߂Â����ꍇ
			if (abs(FXYDDot(V, -V2)) < 0.1f) {
				count2++;
			}
		}

		// ����̏d�S�������̖@���̉E���ɂ������ꍇ
		else if (dir < 0) {

			// ����̐����Ǝ����̐����̃x�N�g�������s�ɋ߂Â����ꍇ
			if (abs(FXYDDot(-V, V1)) < 0.1f) {
				count2++;
			}
		}
	}

	// �����Ɛڂ����Ƃ��̉�]��̍��W�擾.
	// V�@�@�ڐG��������̃x�N�g��
	// V1�@ �ڐG�������_�̍����̃x�N�g��
	// V2�@ �ڐG�������_�̉E���̃x�N�g��
	// fAngle	��]�p�x.

	void RotPos(FXY V, FXY V1, FXY V2, float fAngle)
	{
		if (!IS_FIT_ANGLE(fAngle)) {
			TRACE(_T("�p�x��0.0�`360.0�Ɏ��܂��Ă��܂���[%f]\n"), fAngle);
			ASSERT(false);	// FIT_ANGLE()��ʂ��Ė߂��ꂽ�l������ȍ~�����p����K�v������܂�.
		}

		float rad = 0;	// ���W�A��.
		float cosValue;
		float sinValue;

		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// �����̏d�S������̖@���̍����ɂ������ꍇ
		if (dir > 0) {
			if (FXYDDot(V, -V2) < 0) {
				ag = ag - fAngle;
				rad = -ANGLE(fAngle);
			}
		}

		// ����̏d�S�������̖@���̉E���ɂ������ꍇ
		else if (dir < 0) {
			if (FXYDDot(V, V1) > 0) {
				ag = ag + fAngle;
				rad = ANGLE(fAngle);
			}
		}

		cosValue = cosf(rad);
		sinValue = sinf(rad);

		// ��]�ϊ�
		Pos.x = Axis.x + (Pos.x - Axis.x) * cosValue - (Pos.y - Axis.y) * sinValue;
		Pos.y = Axis.y + (Pos.x - Axis.x) * sinValue + (Pos.y - Axis.y) * cosValue;

		ag = FIT_ANGLE(ag);
	}

	// ��]��̒��_�̍��W�����߂�
	void CalcPoint() {

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

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

	// �`��
	void draw(HDC& hdc) {

		RenderRotRectangle(hdc, Pos, point, ag, Color);
		RenderCircle(hdc, Pos.x, Pos.y, 5, Color);
		if (situation) {
			RenderCircle(hdc, Outpoint[num].x, Outpoint[num].y, 5, Color);
		}
	}
};

// ��`�̃R���X�g���N�^
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// ��
	h = height;		// ����

	ag = angle;	// �p�x(45�x��).
	ag = FIT_ANGLE(ag);

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

	GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 10;	// 0.1�b�Ԋu�ōX�V.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// �e��I�u�W�F�N�g�̏����z�u.
// �u�����z�u���Z�b�g�v�̃{�^�����������炱�̊֐����Ă΂�܂�.
// ���̌�AUpdateObject()��unTimer == 0 �łP�񂾂��Ă΂�܂�.
// �e�}�`�̏����ʒu��������Őݒ肵�Ă�������.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

	// �����̎n�_�ƏI�_�i�R�b�v�j
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

	// ��`�̏�����
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


	// �~�̏�����
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


// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(&bReset, hdc);

}

// �ʒu�X�V
void UpdatePos(bool* PtrbReset, HDC& hdc) {

	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		FXY gravity(0, 0.1f);

		// �~�̈ʒu�X�V
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

		// ��`�̈ʒu�X�V
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
// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

// ��`�ɉ�]��ǉ��i�������j�@�����\�Ɏg�p
#ifdef VECTOR_ADD_RECT_ROT

// �v���g�^�C�v�錾
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̃N���X
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // �����x�N�g��
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// �����̃R���X�g���N�^
SEGMENT::SEGMENT(float spx, float spy, float epx, float epy) {
	Sp.x = spx;
	Sp.y = spy;
	Ep.x = epx;
	Ep.y = epy;

	V = VectorSub(Sp, Ep);

	// �@���x�N�g��
	N.x = -V.y;
	N.y = V.x;

	// ���ꂼ�ꐳ�K��
	VectorNormalize(N);
	VectorNormalize(V);
}

// �~�̃N���X
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
	float rate = 0.6f; // �Փˉ����Ɏg���W��
public:
	BALL(UINT color, float r, float px, float py, float restitution);

	// �����x
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// 
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	// �����Ƃ̏Փ˔���A����
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// �����Ɖ~�̒��S�Ƃ̋����i���ρj
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();

			// �����Ɛ����̗��_�̊O��
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {

				// �߂荞�݂����������v�Z
				float excess = Radius - abs(dist);

				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;

				calcReflect(n);
				Vel *= Restitution;
			}

			// �n�_�Ɖ~�̒��S�Ƃ̋���
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}

			// �I�_�Ɖ~�̒��S�Ƃ̋���
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// �~�Ƃ̏Փ˔���A����
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

	// ���˃x�N�g�������߂�
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// �`��
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// �~�̃R���X�g���N�^
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// ��`�̃N���X
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// ��
	float h = 0.0f;		// ����
	float ag = 0.0f;	// �p�x
	UINT Color;

	FXY point[4];	// rad = 0 ���̊e���_�̍��W
	FXY Outpoint[4];	// rad = ag/pi ���̊e���_�̍��W

	float Restitution = 0.0f; // �����W��

	float state = false;

	float dir = 0.0f; // �O�ς̕����m�F�p

	FXY Axis; // ���̂ɏՓ˂������ɐڐG�����_�̈ʒu��ۊ�

	FXY N[4]; // ��`�̕ӂ̖@���x�N�g��

	int num = 5;

	bool situation = false;

	FXY mv[4]; // �������g�̕ӂ̃x�N�g��

	bool StaticRect = false; // �R�b�v�ƐڐG����_�̐��ɉ����ĕ��̂��Œ肷��t���O

	bool StaticRect2 = false; // ��`�ƐڐG����_�̐��ɉ����ĕ��̂��Œ肷��t���O

	int count2 = 0; // �R�b�v�Ƃ̐ڐG�_�̐�

	int count3 = 0; // ��`�Ƃ̐ڐG�_�̐�

	FXY LOG; // �����Ƃ̏Փ˔���I�����̌v�Z���ʂ�ۊ�

public:

	// �R���X�g���N�^
	RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution);

	const FXY& OUTPOINT(int i) { return Outpoint[i]; }

	// �����x
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// �ړ�
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;

		ag = FIT_ANGLE(ag);

		// �l�p�`�̌`�󂩂��]���l���������_�̈ʒu�����߂�
		CalcPoint();

		count2 = 0;

		count3 = 0;

		StaticRect = false;

		StaticRect2 = false;

	}

	// �����Ƃ̏Փ˔���A����
	void collisionSegment(SEGMENT& seg) {

		const FXY& n = seg.n();
		const FXY& v = seg.v();

		// �����̎n�_�����`��4���_�ւ̃x�N�g��
		FXY v0 = Outpoint[0] - seg.sp();
		FXY v1 = Outpoint[1] - seg.sp();
		FXY v2 = Outpoint[2] - seg.sp();
		FXY v3 = Outpoint[3] - seg.sp();

		// �����Ƌ�`�̒��_�܂ł̋���
		float dist0 = FXYDDot(n, v0);
		float dist1 = FXYDDot(n, v1);
		float dist2 = FXYDDot(n, v2);
		float dist3 = FXYDDot(n, v3);

		// ��`�̒��_�������̍��E�ǂ���ɂ��邩�����Ŕ��f����
		float value0 = FXYDCross(v, v0);
		float value1 = FXYDCross(v, v1);
		float value2 = FXYDCross(v, v2);
		float value3 = FXYDCross(v, v3);

		// ��`�̒��_�����ׂĐ����ɑ΂��ē��������ɂ������ꍇ
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {

			state = true;

			// �����̏I�_�����`��4���_�ւ̃x�N�g��
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();
			
			// ��`�̒��_�����`�̏d�S�܂ł̃x�N�g��
			FXY centerV0 = Pos - Outpoint[0];
			FXY centerV1 = Pos - Outpoint[1];
			FXY centerV2 = Pos - Outpoint[2];
			FXY centerV3 = Pos - Outpoint[3];

			// �����̖@���x�N�g���Ƌ�`�̒��_�����`�̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��p
			float valueN0 = FXYDCross(n, centerV0);
			float valueN1 = FXYDCross(n, centerV1);
			float valueN2 = FXYDCross(n, centerV2);
			float valueN3 = FXYDCross(n, centerV3);

			// �����̃x�N�g���Ƌ�`�̒��_�����`�̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��p
			float valueC0 = FXYDCross(v, centerV0);
			float valueC1 = FXYDCross(v, centerV1);
			float valueC2 = FXYDCross(v, centerV2);
			float valueC3 = FXYDCross(v, centerV3);

			if (value0 > 0) {

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

				// �����Ɛ����̗��_�̊O��
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

			// �����ƐڐG�������_�̐�����ȏ�̏ꍇ���̂��Œ肷��
			if (count3 >= 2) {
				StaticRect = true;
			}

			// �Œ肳��ĂȂ��������]������
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
			//// �n�_�Ɖ~�̒��S�Ƃ̋���
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// �I�_�Ɖ~�̒��S�Ƃ̋���
			//else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
			//	VectorNormalize(v2);
			//	Pos = seg.ep() + v2 * Radius;
			//	calcReflect(v2);
			//	Vel *= Restitution;
			//}

		}
	}

	// ��`�Ƃ̏Փ˔���A����
	void collisionRectangle(RECTANGLE& other) {
		if (this != &other) {

			// �ڐG������`�̕ӂ̃x�N�g��
			FXY ov[4];
			ov[0] = other.Outpoint[1] - other.Outpoint[0];
			ov[1] = other.Outpoint[2] - other.Outpoint[1];
			ov[2] = other.Outpoint[3] - other.Outpoint[2];
			ov[3] = other.Outpoint[0] - other.Outpoint[3];

			// [����][����] ���_�ւ̃x�N�g��
			FXY inside[4][4];
			for (int i = 0; i < 4; i++) {
				inside[0][i] = Outpoint[i] - other.Outpoint[0];
				inside[1][i] = Outpoint[i] - other.Outpoint[1];
				inside[2][i] = Outpoint[i] - other.Outpoint[2];
				inside[3][i] = Outpoint[i] - other.Outpoint[3];
			}


			// [����][����] �O��
			float value[4][4];
			for (int i = 0; i < 4; i++) {
				value[0][i] = FXYDCross(ov[0],inside[0][i]);
				value[1][i] = FXYDCross(ov[1],inside[1][i]);
				value[2][i] = FXYDCross(ov[2],inside[2][i]);
				value[3][i] = FXYDCross(ov[3],inside[3][i]);
			}

			// ������4�̒��_���玩�����g�̏d�S�ւ̃x�N�g��
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

			// �����̒��_4�ɑ΂��ďՓ˂��Ă��邩���肷��
			for (int i = 0; i < 4; i++) {
				if (value[0][i] <= 0 && value[1][i] <= 0 && value[2][i] <= 0 && value[3][i] <= 0) {

					situation = true;

					// �����ƒ��_�Ƃ̋������ŏ��̐����̔ԍ����L�^���邽�߂ɒu�����i�v�C���j
					int min = 0;

					for (int j = 0; j < 4; j++) {
						// �����̐����Ƒ���̂ǂ̐����ƌ����������𔻒�
						if (CrossSegment(ov[j], -centerV[i], other.Outpoint[j], Pos)) {

							// ���̏ꍇ���ꂼ��ڐG�_���J�E���g����
							count2++;
							other.count2++;
							min = j;
						}
					}

					// ����̐���4�Ƒ���̋�`�ɐN�����������̒��_�Ƃ̋���
					float dist[4];
					
					// ���̌v�Z
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

					// ��`�ƐڐG���A��]����悤�ȏꍇ�ɕK�v�ȐڐG��������̐����̖@���x�N�g���Ǝ����̒��_���玩���̏d�S�܂ł̃x�N�g���̊O�ς̕����𔻒肷��̂ɗp����
					float valueN[4];

					for (int i = 0; i < 4; i++) {
						valueN[i] = FXYDCross(-other.N[min], centerV[i]);
					}

					num = i;

					dir = valueN[i];

					// 
					RotPosCount(-ov[min], mv[i], mv[(i + 3) % 4]);

					// �����x�̋t�x�N�g���i�v���P�j
					FXY H;
					H.x = 0;
					H.y = -1;
					dir = FXYDCross(H, -centerV[i]);

					//other.Color = RGB(255, 255, 0);
					//Color = RGB(255, 255, 0);
					
					// ��̋�`���r���ĐG��Ă��钸�_�����ꂼ��2�ȏ�̏ꍇ
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

					//// �n�_�Ɖ~�̒��S�Ƃ̋���
					//
					//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
					//	VectorNormalize(v1);
					//	Pos = seg.sp() + v1 * Radius;
					//	calcReflect(v1);
					//	Vel *= Restitution;
					//}
					//// �I�_�Ɖ~�̒��S�Ƃ̋���
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

	// �d�S���ӂ̊O�ɏo�Ă��邩
	bool CenterRot(RECTANGLE& other, int min, int i) {

		// ���̓��m���Œ肵�Ă����ꍇ
		if (count2 + count3 >= 2 && other.count2 + other.count3 >= 2) {
			TRACE(_T("CenterRot ME Sum: %d\n"), count3 + count2);
			TRACE(_T("CenterRot OTHER Sum: %d\n"), other.count2 + other.count3);
			return false;
		}

		// ����������̏�ɏd�Ȃ�ꍇ�Ŏ����̏d�S������̐����̗��[�̈ʒu����͂ݏo���Ă����ꍇ
		if (count2 + count3 < 2) {
			if (Pos.x > other.Outpoint[min].x || Pos.x < other.Outpoint[(min + 1) % 4].x) {
				if (Outpoint[(i + 1) % 4].y - other.Outpoint[min].y < 0 || Outpoint[(i + 3) % 4].y - other.Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// ���肪�����̏�ɏd�Ȃ�ꍇ�ő���̏d�S�������̐����̗��[�̈ʒu����͂ݏo���Ă����ꍇ
		if (other.count2 + other.count3 < 2) {
			if (other.Pos.x > Outpoint[min].x || other.Pos.x < Outpoint[(min + 1) % 4].x) {
				if (other.Outpoint[(i + 1) % 4].y - Outpoint[min].y < 0 || other.Outpoint[(i + 3) % 4].y - Outpoint[(min + 1) % 4].y < 0) {
					return true;
				}
			}
		}

		// �s���S
		return true;
	}

	// �Q�̐�����������Ă��邩����
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

		// 0 <= t <=1�̏ꍇ
		if (t1 < 0 || t1 > 1 || t2 < 0 || t2 > 1) {
			return false;
		}

		return true;
	}

	// ���˃x�N�g�����v�Z
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// �z����ōŏ��l������ԍ���Ԃ�
	int minValue(float nums[], int n) {
		float min_value; // �ő�l
		int i;
		int j = 0;
		// nums[0]���ő�l�Ɖ��肷��
		min_value = nums[0];

		for (i = 0; i < n; i++) {
			if (nums[i] < min_value) {
				/* �ő�l����nums[i]�̕����傫����΍ő�l���X�V */
				min_value = nums[i];
				j = i;
			}
		}

		return j;
	}

	// ��`�ƐڐG���ĉ�]�����㑊��̐����ɐڋ߂����ꍇ�ڐG�_��������
	// V�@�@�ڐG��������̃x�N�g��
	// V1�@ �ڐG�������_�̍����̃x�N�g��
	// V2�@ �ڐG�������_�̉E���̃x�N�g��

	void RotPosCount(FXY V, FXY V1, FXY V2)
	{
		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// �����̏d�S������̖@���̍����ɂ������ꍇ
		if (dir > 0) {

			// ����̐����Ǝ����̐����̃x�N�g�������s�ɋ߂Â����ꍇ
			if (abs(FXYDDot(V, -V2)) < 0.1f) {
				count2++;
			}
		}

		// ����̏d�S�������̖@���̉E���ɂ������ꍇ
		else if (dir < 0) {

			// ����̐����Ǝ����̐����̃x�N�g�������s�ɋ߂Â����ꍇ
			if (abs(FXYDDot(-V,  V1)) < 0.1f) {
				count2++;
			}
		}
	}

	// �����Ɛڂ����Ƃ��̉�]��̍��W�擾.
	// V�@�@�ڐG��������̃x�N�g��
	// V1�@ �ڐG�������_�̍����̃x�N�g��
	// V2�@ �ڐG�������_�̉E���̃x�N�g��
	// fAngle	��]�p�x.

	void RotPos(FXY V, FXY V1, FXY V2, float fAngle)
	{
		if (!IS_FIT_ANGLE(fAngle)) {
			TRACE(_T("�p�x��0.0�`360.0�Ɏ��܂��Ă��܂���[%f]\n"), fAngle);
			ASSERT(false);	// FIT_ANGLE()��ʂ��Ė߂��ꂽ�l������ȍ~�����p����K�v������܂�.
		}

		float rad = 0;	// ���W�A��.
		float cosValue;
		float sinValue;

		VectorNormalize(V);
		VectorNormalize(V1);
		VectorNormalize(V2);

		// �����̏d�S������̖@���̍����ɂ������ꍇ
		if (dir > 0) {
			if (FXYDDot(V, -V2) < 0) {
				ag = ag - fAngle;
				rad = -ANGLE(fAngle);
			}
		}

		// ����̏d�S�������̖@���̉E���ɂ������ꍇ
		else if(dir < 0) {
			if (FXYDDot(V, V1) > 0) {
				ag = ag + fAngle;
				rad = ANGLE(fAngle);
			}
		}

		cosValue = cosf(rad);
		sinValue = sinf(rad);

		// ��]�ϊ�
		Pos.x = Axis.x + (Pos.x - Axis.x) * cosValue - (Pos.y - Axis.y) * sinValue;
		Pos.y = Axis.y + (Pos.x - Axis.x) * sinValue + (Pos.y - Axis.y) * cosValue;

		ag = FIT_ANGLE(ag);
	}

	// ��]��̒��_�̍��W�����߂�
	void CalcPoint() {

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

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

	// �`��
	void draw(HDC& hdc) {

		RenderRotRectangle(hdc, Pos, point, ag, Color);
		RenderCircle(hdc, Pos.x, Pos.y, 5, Color);
		if (situation) {
			RenderCircle(hdc, Outpoint[num].x, Outpoint[num].y, 5, Color);
		}
	}
};

// ��`�̃R���X�g���N�^
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// ��
	h = height;		// ����

	ag = angle;	// �p�x(45�x��).
	ag = FIT_ANGLE(ag);

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

	GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 10;	// 0.1�b�Ԋu�ōX�V.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// �e��I�u�W�F�N�g�̏����z�u.
// �u�����z�u���Z�b�g�v�̃{�^�����������炱�̊֐����Ă΂�܂�.
// ���̌�AUpdateObject()��unTimer == 0 �łP�񂾂��Ă΂�܂�.
// �e�}�`�̏����ʒu��������Őݒ肵�Ă�������.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

	// �����̎n�_�ƏI�_�i�R�b�v�j
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

	// ��`�̏�����
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

	
	// �~�̏�����
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


// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(&bReset, hdc);

}
// �ʒu�X�V
void UpdatePos(bool* PtrbReset, HDC& hdc) {

	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		FXY gravity(0, 0.1f);

		// �~�̈ʒu�X�V
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity); // �����x
				ball.move(); // �ړ�
				for (BALL& other : balls) {
					ball.collisionBall(other); // �~�Ƃ̏Փ˔���
				}
				for (SEGMENT& segment : segments) {
					ball.collisionSegment(segment); // �����Ƃ̏Փ˔���
				}
				ball.draw(hdc); // �`��
			}
		}

		// ��`�̈ʒu�X�V
		{
			for (RECTANGLE& rect : rects) {
				rect.applyForce(gravity);  // �����x
				rect.move();  // �ړ�

				for (SEGMENT& segment : segments) {
					rect.collisionSegment(segment);  // �����Ƃ̏Փ˔���
				}
				for (RECTANGLE& other : rects) {
					rect.collisionRectangle(other); // ��`�Ƃ̏Փ˔���
				}
				rect.draw(hdc); // �`��
			}
		}

	}
}

// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

// ��`�ƃR�b�v�Ƃ̃R���W��������
#ifdef VECTOR_ADD_RECT
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̃N���X
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // �����x�N�g��
	FXY V;

public:
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& v() { return V; }
	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// �����̃R���X�g���N�^
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

// �~�̃N���X
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

	// �����Ƃ̏Փ˔���A����
	void collisionSegment(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// �����Ɖ~�̒��S�Ƃ̋����i���ρj
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();
			// �����Ɛ����̗��_�̊O��
			if (FXYDCross(n, v1) < 0 && FXYDCross(n, v2) > 0) {
				float excess = Radius - abs(dist);
				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;
				calcReflect(n);
				Vel *= Restitution;
			}
			// �n�_�Ɖ~�̒��S�Ƃ̋���
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}
			// �I�_�Ɖ~�̒��S�Ƃ̋���
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// �~�Ƃ̏Փ˔���A����
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

// �~�̃R���X�g���N�^
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// ��`�̃N���X
class RECTANGLE {
	FXY Pos, Vel, Acc;
	float w = 0.0f;		// ��
	float h = 0.0f;		// ����
	float ag = 0.0f;	// �p�x
	UINT Color;

	FXY point[4];	// rad = 0 ���̊e���_�̍��W
	FXY Outpoint[4];	// rad = ag/pi ���̊e���_�̍��W

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

		point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
		point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
		point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
		point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

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
	// �����Ƃ̏Փ˔���A����
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

		// �����Ɖ~�̒��S�Ƃ̋����i���ρj
		if (value0 > 0 || value1 > 0 || value2 > 0 || value3 > 0) {
			FXY v4 = Outpoint[0] - seg.ep();
			FXY v5 = Outpoint[1] - seg.ep();
			FXY v6 = Outpoint[2] - seg.ep();
			FXY v7 = Outpoint[3] - seg.ep();

			if (value0 > 0) {
				// �����Ɛ����̗��_�̊O��
				if (FXYDCross(n, v0) < 0 && FXYDCross(n, v4) > 0) {
					FXY a = Outpoint[0] - Outpoint[3];
					float excess2 = FXYDDot(n, a) + dist3;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			if (value1 > 0) {
				// �����Ɛ����̗��_�̊O��
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
				// �����Ɛ����̗��_�̊O��
				if (FXYDCross(n, v2) < 0 && FXYDCross(n, v6) > 0) {
					FXY a = Outpoint[2] - Outpoint[1];
					float excess2 = FXYDDot(n, a) + dist1;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			if (value3 > 0) {
				// �����Ɛ����̗��_�̊O��
				if (FXYDCross(n, v3) < 0 && FXYDCross(n, v7) > 0) {
					FXY a = Outpoint[3] - Outpoint[2];
					float excess2 = FXYDDot(n, a) + dist2;
					Pos -= n * excess2;

					calcReflect(n);
					Vel *= Restitution;
				}
			}
			
			point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
			point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
			point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
			point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).

			GetRotPos(Outpoint, Pos, point, COUNTOF(point), ag);

			//// �n�_�Ɖ~�̒��S�Ƃ̋���
			//else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
			//	VectorNormalize(v1);
			//	Pos = seg.sp() + v1 * Radius;
			//	calcReflect(v1);
			//	Vel *= Restitution;
			//}
			//// �I�_�Ɖ~�̒��S�Ƃ̋���
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

// ��`�̃R���X�g���N�^
RECTANGLE::RECTANGLE(UINT color, float px, float py, float width, float height, float angle, float restitution) {
	w = width;			// ��
	h = height;		// ����

	ag = angle;	// �p�x(45�x��).

	Restitution = restitution;

	Color = color;

	Pos.x = px;
	Pos.y = py;

	point[0] = { Pos.x - w / 2.0f, Pos.y - h / 2.0f };	// ����(x, y).
	point[1] = { Pos.x - w / 2.0f, Pos.y + h / 2.0f };	// ����(x, y).
	point[2] = { Pos.x + w / 2.0f, Pos.y + h / 2.0f };	// �E��(x, y).
	point[3] = { Pos.x + w / 2.0f, Pos.y - h / 2.0f };	// �E��(x, y).
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 10;	// 0.1�b�Ԋu�ōX�V.
}

vector<SEGMENT> segments;
vector<BALL> balls;
vector<RECTANGLE> rects;

// �e��I�u�W�F�N�g�̏����z�u.
// �u�����z�u���Z�b�g�v�̃{�^�����������炱�̊֐����Ă΂�܂�.
// ���̌�AUpdateObject()��unTimer == 0 �łP�񂾂��Ă΂�܂�.
// �e�}�`�̏����ʒu��������Őݒ肵�Ă�������.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

	// �����̎n�_�ƏI�_�i�R�b�v�j
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

	// ��`�̏�����
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
	// �~�̏�����
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


// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(&bReset, hdc);

}

// �ʒu�X�V
void UpdatePos(bool* PtrbReset, HDC& hdc) {
	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		FXY gravity(0, 0.2f);

		// �~�̈ʒu�X�V
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

		// ��`�̈ʒu�X�V
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
// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

// ���������݂悤�Ƃ�������
#ifdef TEST00

float FXYDDot(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̍\����

class SEGMENT {
	FXY Sp;
	FXY Ep;
	FXY N; // �����x�N�g��
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

BALL ball(100, 200, 0, 0, 0.03f, 0.3f);	// �~.

template
<
	typename TYPE,
	std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE])
{
	return SIZE;
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 100;	// 0.1�b�Ԋu�ōX�V.
}

void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

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

// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(PtrbReset);

	// �`��
	Draw(hdc);
}

// �`�悷��.
void Draw(HDC& hdc) {
	// ���W(x, y)���S�ɔ��ar�̉~��`�悷��.
	RenderCircle(hdc, ball.getPos().x, ball.getPos().y, ball.getRad(), ball.getRGB());
}

// �ʒu�X�V
void UpdatePos(bool* PtrbReset) {
	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		// �~�̈ʒu�X�V
		{
			ball.manual();
			ball.move();
			ball.collision(segment2);

		}
	}

}
// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

// �~���m�̃R���W���������@�����\�Ɏg�p
#ifdef VECTOR_BALL

// �v���g�^�C�v�錾
float FXYDDot(FXY sLhs, FXY sRhs);
float FXYDCross(FXY sLhs, FXY sRhs);
void VectorNormalize(FXY& sXy);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̃N���X
class SEGMENT {
	FXY Sp, Ep;
	FXY N; // �����x�N�g��
public:
	// �R���X�g���N�^
	SEGMENT(float spx, float spy, float epx, float epy);

	const FXY& n() { return N; }
	const FXY& sp() { return Sp; }
	const FXY& ep() { return Ep; }
};

// �R���X�g���N�^
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

// �~�̃N���X
class BALL {
	FXY Pos, Vel, Acc;
	float Radius;
	UINT Color;
	float Restitution = 0.0f;
public:
	// �R���X�g���N�^
	BALL(UINT color, float r, float px, float py, float restitution);

	// �����x
	void applyForce(const FXY& force) {
		Acc += force;
	}

	// �ړ�
	void move() {
		Vel += Acc;
		Pos += Vel;
		Acc *= 0;
	}

	void collision(SEGMENT& seg) {
		const FXY& n = seg.n();
		FXY v1 = Pos - seg.sp();
		float dist = FXYDDot(n, v1);
		// �����Ɖ~�̒��S�Ƃ̋����i���ρj
		if (abs(dist) < Radius) {
			FXY v2 = Pos - seg.ep();
			// �����Ɛ����̗��_�̊O��
			if (FXYDCross(n,v1) < 0 && FXYDCross(n, v2) > 0) {
				float excess = Radius - abs(dist);
				if (dist > 0)
					Pos += n * excess;
				else
					Pos -= n * excess;
				calcReflect(n);
				Vel *= Restitution;
			}
			// �n�_�Ɖ~�̒��S�Ƃ̋���
			else if (v1.x * v1.x + v1.y * v1.y < Radius * Radius) {
				VectorNormalize(v1);
				Pos = seg.sp() + v1 * Radius;
				calcReflect(v1);
				Vel *= Restitution;
			}
			// �I�_�Ɖ~�̒��S�Ƃ̋���
			else if (v2.x * v2.x + v2.y * v2.y < Radius * Radius) {
				VectorNormalize(v2);
				Pos = seg.ep() + v2 * Radius;
				calcReflect(v2);
				Vel *= Restitution;
			}
		}
	}

	// �~���m�̏Փ˔���ƏՓˉ���
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

	// ���˃x�N�g�������߂�
	void calcReflect(const FXY& n) {
		float dot = FXYDDot(n, -Vel);
		Vel = n * dot * 2 + Vel;
	}

	// �`��
	void draw(HDC& hdc) {
		RenderCircle(hdc, Pos.x, Pos.y, Radius, Color);
	}
};

// �R���X�g���N�^
BALL::BALL(UINT color, float r, float px, float py, float restitution) {
	Color = color; Radius = r;
	Pos.x = px; Pos.y = py;
	Restitution = restitution;
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 10;	// 0.1�b�Ԋu�ōX�V.
}

vector<SEGMENT> segments;
vector<BALL> balls;

// �e��I�u�W�F�N�g�̏����z�u.
// �u�����z�u���Z�b�g�v�̃{�^�����������炱�̊֐����Ă΂�܂�.
// ���̌�AUpdateObject()��unTimer == 0 �łP�񂾂��Ă΂�܂�.
// �e�}�`�̏����ʒu��������Őݒ肵�Ă�������.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

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
		// ������I���ɂ����ResetObject���Ă΂ꂽ���ɔz��̒��g������
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

// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(&bReset, hdc);

}

// �ʒu�X�V
void UpdatePos(bool* PtrbReset, HDC& hdc) {
	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		FXY gravity(0, 0.2f);

		// �~�̈ʒu�X�V
		{
			for (BALL& ball : balls) {
				ball.applyForce(gravity); // �����x
				ball.move(); // �ړ�
				for (BALL& other : balls) {
					ball.collisionBall(other); // �~�Ƃ̏Փ˔���
				}
				for (SEGMENT& segment : segments) {
					ball.collision(segment); // �����Ƃ̏Փ˔���
				}
				ball.draw(hdc); // �`��
			}

		}
	}

}
// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

// �z�z���ꂽ�v���O��������Œ���֐����������
#ifdef BASE
float FXYDDot(FXY sLhs, FXY sRhs);

const FXY g_sCupPoint[4] = { { 100.0f, 100.0f },{ 100.0f, 600.0f },{ 600.0f, 600.0f },{ 600.0f, 100.0f } }; // �R�b�v�̎l���̍��W.

// �����̍\����
struct SEGMENT {
public:
	FXY Sp;
	FXY Ep;
	FXY N; // �����x�N�g��

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

// ��F�~�̍\����.
struct S_CIRCLE {
public:
	float r;
	UINT rgb;

	// ���W(x, y)�Ɏl�p�`��`�悷�� ���W���S�͎l�p�`�̏d�S�ɐݒ�
	FXY center;

	S_CIRCLE();
};

// �R���X�g���N�^
S_CIRCLE::S_CIRCLE() {
	center.x = 100;
	center.y = 100;
	r = 10;
	rgb = RGB(144, 238, 144);
}

S_CIRCLE g_sCircle[5];	// �~.

// ��F�l�p�`�̍\����.
struct S_SQUARE {
public:
	float w;		// ��
	float h;		// ����

	float angle;			

	UINT rgb;
	// ���W(x, y)�Ɏl�p�`��`�悷�� ���W���S�͎l�p�`�̏d�S�ɐݒ�
	FXY center;

	FXY point[4];	// rad = 0 ���̊e���_�̍��W

	S_SQUARE();
};

// �R���X�g���N�^
S_SQUARE::S_SQUARE() {
	w = 50;			// ��
	h = 100;		// ����

	angle = 45.0f;	// �p�x(45�x��).

	rgb = RGB(144, 238, 144);

	center.x = 400;
	center.y = 200;

	point[0] = { center.x - w / 2.0f, center.y - h / 2.0f };	// ����(x, y).
	point[1] = { center.x - w / 2.0f, center.y + h / 2.0f };	// ����(x, y).
	point[2] = { center.x + w / 2.0f, center.y + h / 2.0f };	// �E��(x, y).
	point[3] = { center.x + w / 2.0f, center.y - h / 2.0f };	// �E��(x, y).
}

S_SQUARE g_sSquare;	// �l�p�`.

// ��F�񓙕ӎO�p�`�̍\����.
struct S_TRIANGLE {
public:
	float w;		// ���
	float h;		// ����

	// �}�`�̉�]�p���w�肵�܂�.
	// �p�x�͕K��0.0�`360.0�Ɏ��߂Ă�������(�͈͂��O���ƌv�Z�덷���傫���Ȃ邽��).
	// FIT_ANGLE()�̃}�N����ʂ��Δ͈͂Ɏ��߂�l��Ԃ��܂�.
	// ��) FIT_ANGLE(362.0f) �� 2.0f ��߂��܂�.
	//     FIT_ANGLE(-10.0f) �� 350.0f ��߂��܂�.
	float angle;

	UINT rgb;
	// ���W(x, y)�ɓ񓙕ӎO�p�`��`�悷�� ���W���S�͓񓙕ӎO�p�`�̏d�S�ɐݒ�
	FXY center;

	FXY point[3];	// rad = 0 ���̊e���_�̍��W

	S_TRIANGLE();
};

// �R���X�g���N�^
S_TRIANGLE::S_TRIANGLE() {
	w = 100.0f;			// ���
	h = 70.0f;			// ����

	angle = 30.0f;			// �p�x(30�x��).

	rgb = RGB(144, 238, 144);

	center.x = 400;
	center.y = 200;

	point[0] = { center.x + w / 2.0f, center.y - h / 3.0f };
	point[1] = { center.x - w / 2.0f, center.y - h / 3.0f };
	point[2] = { center.x,	          center.y + (2.0f * h) / 3.0f };
}

S_TRIANGLE g_sTriangle;	// �񓙕ӎO�p�`.

template
<
	typename TYPE,
	std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE])
{
	return SIZE;
}

// �����V�[�h���擾����.
// �V�[�h�l���Œ艻����ƁA���񓯂��������������܂��B
// �܂薈�񓯂��z�u��Ԃ���X�^�[�g���܂�.
UINT GetRandomSeed()
{
	UINT unSeed = 12345;

	// �������V�[�h�l���Œ艻�������ꍇ�́����R�����g�A�E�g���Ă�������.
	unSeed = static_cast<UINT>(time(NULL));	// �����炪�L�����Ɩ���قȂ闐�����������܂�.

	return unSeed;
}


// �X�V�p�x���擾����.
// �R�}����œ��삳���邱�Ƃ��ł��܂�.
UINT GetUpdateMsec()
{
	// �X�V����(1000��1�b�A�������X�V��������ΐ�����傫������).
	// �����Ԓʂ�Ɍv�Z����K�v�͂���܂���.
	return 10;	// 0.1�b�Ԋu�ōX�V.
}


// �e��I�u�W�F�N�g�̏����z�u.
// �u�����z�u���Z�b�g�v�̃{�^�����������炱�̊֐����Ă΂�܂�.
// ���̌�AUpdateObject()��unTimer == 0 �łP�񂾂��Ă΂�܂�.
// �e�}�`�̏����ʒu��������Őݒ肵�Ă�������.
void ResetObject()
{
	TRACE(_T("Reset Object\n"));	// printf�̑���ɗ��p�ł��܂��B�i�o�̓E�C���h�E�ɕ\������܂��j.

	{
		for (int i = 0; i < array_length(g_sCircle); i++) {
			// �~�̏����ʒu�������_���Őݒ�.
			while (1) {
				g_sCircle[i].center.x = RandomRangeF(200.0f, 500.0f);	// 200�`500 �̊Ԃ̃����_���Ȑ���.
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
		// �l�p�`�̏����ʒu�������_���Őݒ�.
		g_sSquare.w = RandomRangeF(50.0f, 200.0f);
		g_sSquare.h = RandomRangeF(50.0f, 200.0f);
		g_sSquare.angle = RandomRangeF(0.0f, 360.0f);

		g_sSquare.center.x = RandomRangeF(200.0f, 500.0f);	// 200�`500 �̊Ԃ̃����_���Ȑ���.
		g_sSquare.center.y = RandomRangeF(100.0f, 200.0f);
		g_sSquare.rgb = RGB(30, 144, 255);	// �F.
	}

	{
		// �񓙕ӎO�p�`�̏����ʒu�������_���Őݒ�.
		g_sTriangle.w = RandomRangeF(50.0f, 200.0f);
		g_sTriangle.h = RandomRangeF(50.0f, 200.0f);
		g_sTriangle.angle = RandomRangeF(0.0f, 360.0f);

		g_sTriangle.center.x = RandomRangeF(200.0f, 500.0f);	// 200�`500 �̊Ԃ̃����_���Ȑ���.
		g_sTriangle.center.y = RandomRangeF(100.0f, 200.0f);
		g_sTriangle.rgb = RGB(250, 140, 0);	// �F.
	}

}

void DetectColiderSphere(SEGMENT& seg);
void CalcPoint(int x);
void Draw(HDC& hdc);
void UpdatePos(bool* PtrbReset);

// �e��I�u�W�F�N�g�̍X�V.
// �u�R���W���������v�̃{�^���������ƁA���Ԋu�Ŗ{�֐����Ă΂�܂�(������x�����Ǝ~�܂�܂�).
// �Ă΂��^�C�~���O��ς������Ƃ��� GetUpdateMsec() ��ύX���Ă�������(������Ԃ�0.1�b�Ԋu�ł�).
// �ʂɎ��Ԃɍ��킹�ď��������A��u�ŗ������ĉ�����ԂƂ��Ă���肠��܂���.
// �����@���ɑ����ė�������K�v������܂���(������ۂ��v�Z�ŏ\���ł�).
// �����FunTimer �Ă΂��x�ɃJ�E���g�A�b�v���܂�(0�͏������).
void UpdateObject(HDC& hdc, UINT unTimer)
{
	bool bReset = (unTimer == 0);
	bool* PtrbReset;
	PtrbReset = &bReset;

	// �ʒu�X�V
	UpdatePos(PtrbReset);

	// �`��
	Draw(hdc);
}

// �R�b�v�̃R���W�������� (�~)
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
			g_sCircle[i].center.y += 2.0f;	// ����.
		}
		//if (g_sCircle[i].center.y + g_sCircle[i].r >= g_sCupPoint[2].y) {
		//	g_sCircle[i].center.y += 0.0f;
		//}
		//else {
		//	g_sCircle[i].center.y += 2.0f;	// ����.
		//}
	}
}

// �~�̎x�z�̈�
void SphereBoundary() {
	
}

// ���_�ʒu�v�Z�i���t���[���j
void CalcPoint(int x) {
	// ���_���O�̎�
	if (x == 3) {
		g_sTriangle.point[0] = { g_sTriangle.center.x + g_sTriangle.w / 2.0f, g_sTriangle.center.y - g_sTriangle.h / 3.0f };
		g_sTriangle.point[1] = { g_sTriangle.center.x - g_sTriangle.w / 2.0f, g_sTriangle.center.y - g_sTriangle.h / 3.0f };
		g_sTriangle.point[2] = { g_sTriangle.center.x,	          g_sTriangle.center.y + (2.0f * g_sTriangle.h) / 3.0f };

	}
	// ���_���l�̎�
	else if (x == 4) {
		g_sSquare.point[0] = { g_sSquare.center.x - g_sSquare.w / 2.0f, g_sSquare.center.y - g_sSquare.h / 2.0f };	// ����(x, y).
		g_sSquare.point[1] = { g_sSquare.center.x - g_sSquare.w / 2.0f, g_sSquare.center.y + g_sSquare.h / 2.0f };	// ����(x, y).
		g_sSquare.point[2] = { g_sSquare.center.x + g_sSquare.w / 2.0f, g_sSquare.center.y + g_sSquare.h / 2.0f };	// �E��(x, y).
		g_sSquare.point[3] = { g_sSquare.center.x + g_sSquare.w / 2.0f, g_sSquare.center.y - g_sSquare.h / 2.0f };	// �E��(x, y).
	}
}

// �`�悷��.
void Draw(HDC& hdc) {
	// �O�p�`��`�悷��.
	RenderRotTriangle(hdc, g_sTriangle.center, g_sTriangle.point, g_sTriangle.angle, g_sTriangle.rgb);

	// �l�p�`��`�悷��.
	RenderRotRectangle(hdc, g_sSquare.center, g_sSquare.point, g_sSquare.angle, g_sSquare.rgb);

	// ���W(x, y)���S�ɔ��ar�̉~��`�悷��.

	for (int i = 0; i < array_length(g_sCircle); i++) {
		RenderCircle(hdc, g_sCircle[i].center.x, g_sCircle[i].center.y, g_sCircle[i].r, g_sCircle[i].rgb);
	}
}

// �ʒu�X�V
void UpdatePos(bool* PtrbReset) {
	// ���Z�b�g����Ȃ���΍��W���X�V����.
	if (&PtrbReset) {

		// �񓙕ӎO�p�`�̈ʒu�X�V
		{
			g_sTriangle.angle = FIT_ANGLE(g_sTriangle.angle);		// �K�{�F0.0f�`360.0f�̊p�x�Ɏ��߂�.

			CalcPoint(3);
			g_sTriangle.center.y += 2.0f;	// ����.
		}
	}
	if (&PtrbReset) {

		// �l�p�`�̈ʒu�X�V
		{
			g_sSquare.angle = FIT_ANGLE(g_sSquare.angle);		// �K�{�F0.0f�`360.0f�̊p�x�Ɏ��߂�.

			CalcPoint(4);
			g_sSquare.center.y += 2.0f;	// ����.
		}
	}
	if (&PtrbReset) {

		// �~�̈ʒu�X�V
		{
			//DetectColiderSphere(segment1);
			DetectColiderSphere(segment2);
			//DetectColiderSphere(segment3);
		}
	}

}
// �R�b�v�̕`��.
// ������͕ύX�̕K�v�͂Ȃ��͂�.
void RenderCup(HDC& hdc)
{
	// �R�b�v�̕`��
	{
		const int pointCount = sizeof(g_sCupPoint) / sizeof(g_sCupPoint[0]);
		// �`��
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

/* ������  ������ �����܂ł�ҏW���Ă������� ������  ������ */

// �����擾.
int Random(int nMax)
{
	return rand() % (nMax + 1);
}

// �����擾.
int RandomRange(int nMin, int nMax)
{
	if (nMax >= nMin) {
		TRACE(_T("�ő�l���ŏ��l���傫���Ȃ�悤�ɐݒ肵�Ă�������\n"));
		ASSERT(false);
		return nMin;
	}
	UINT nRnd = static_cast<UINT>(nMax - nMin + 1);
	return (nMin + rand() % nRnd);
}

// �����擾.
float RandomF(float fMax)
{
	return static_cast<float>(rand() % static_cast<UINT>(fMax + 1));
}

// �����擾.
float RandomRangeF(float fMin, float fMax)
{
	if (fMin >= fMax) {
		TRACE(_T("�ő�l���ŏ��l���傫���Ȃ�悤�ɐݒ肵�Ă�������\n"));
		ASSERT(false);
		return fMin;
	}
	UINT nRnd = static_cast<UINT>(fMax - fMin + 1);
	return static_cast<float>(fMin + rand() % nRnd);
}


// �Q�n�_�Ԃ̋���.
float CalcDistance(FXY sLhs, FXY sRhs)
{
	FXY sSub = sRhs - sLhs;
	return sqrtf(sSub.x * sSub.x + sSub.y * sSub.y);
}

// �x�N�g���̃T�C�Y.
float VectorLength(FXY sXy)
{
	return sqrtf(sXy.x * sXy.x + sXy.y * sXy.y);
}

// �x�N�g���̍�
FXY VectorSub(FXY sFrom, FXY sTo)
{
	return sTo - sFrom;
}

// �x�N�g�����K��.
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

// �x�N�g���̓���.
float FXYDDot(FXY sLhs, FXY sRhs)
{
	return (sLhs.x * sRhs.x) + (sLhs.y * sRhs.y);
}

// �x�N�g���̊O��.
float FXYDCross(FXY sLhs, FXY sRhs)
{
	return (sLhs.x * sRhs.y) - (sLhs.y * sRhs.x);
}


// ��]��̍��W�擾.
// pOutput	��]�v�Z��̊e���_�̍��W�l�z��.
// center	��]�̒��S���W.
// pPos		��]�v�Z�O�̊e���_�̍��W�l�z��.
// nNum		���W�l�z��̌�(�O�p�`�Ȃ�3, �l�p�`�Ȃ�4��n��).
// fAngle	��]�p�x.
void GetRotPos(FXY* pOutput, FXY center, FXY* pPos, int nNum, float fAngle)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("�p�x��0.0�`360.0�Ɏ��܂��Ă��܂���[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()��ʂ��Ė߂��ꂽ�l������ȍ~�����p����K�v������܂�.
	}
	float rad = ANGLE(fAngle);	// ���W�A��.
	float cosValue = cosf(rad);
	float sinValue = sinf(rad);
	for (int i = 0; i < nNum; i++) {
		// ��]�ϊ�
		pOutput[i].x = center.x + (pPos[i].x - center.x) * cosValue - (pPos[i].y - center.y) * sinValue;
		pOutput[i].y = center.y + (pPos[i].x - center.x) * sinValue + (pPos[i].y - center.y) * cosValue;
	}
}

// ���p�`�̕`��.
void RenderPolygon(HDC hdc, POINT* pPos, int nNum, UINT rgb)
{
	HBRUSH hBrush = CreateSolidBrush(rgb);
	HGDIOBJ hOldObj = SelectObject(hdc, hBrush);	// �h��Ԃ��F�ݒ�
	SetPolyFillMode(hdc, WINDING);
	Polygon(hdc, pPos, nNum);
	SelectObject(hdc, hOldObj);	// ���̐F�ɕ��A
	DeleteObject(hBrush);
}

// �O�p�`�̕`��.
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

// ��]�����O�p�`�̕`��.
// center	��]�̒��S.
// pPos		�e���_���W.
// fAngle	��]�p(0.0�`360.0).
// rgb		�O�p�`�̐F.
void RenderRotTriangle(HDC hdc, FXY& center, FXY* pPos, float fAngle, UINT rgb)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("�p�x��0.0�`360.0�Ɏ��܂��Ă��܂���[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()��ʂ��Ė߂��ꂽ�l������ȍ~�����p����K�v������܂�.
	}
	
	// ��]��̍��W���擾����.
	FXY radPos[3];
	GetRotPos(radPos, center, pPos, COUNTOF(radPos), fAngle);
	
	POINT drawPoint[3];		// �p�x���l�������`�掞�̍��W(�`��API�̓s���Ő����ɂȂ��Ă��܂�)
	for (int i = 0; i < 3; i++) {
		drawPoint[i] = { static_cast<LONG>(radPos[i].x), static_cast<LONG>(radPos[i].y) };
	}

	// �O�p�`�̕`��
	RenderTriangle(hdc, drawPoint, rgb);
}


// �l�p�`�̕`��.
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

// ��]�����l�p�`�̕`��.
// center	��]�̒��S.
// pPos		�e���_���W.
// fAngle	��]�p(0.0�`360.0).
// rgb		�l�p�`�̐F.
void RenderRotRectangle(HDC hdc, FXY& center, FXY* pPos, float fAngle, UINT rgb)
{
	if (!IS_FIT_ANGLE(fAngle)) {
		TRACE(_T("�p�x��0.0�`360.0�Ɏ��܂��Ă��܂���[%f]\n"), fAngle);
		ASSERT(false);	// FIT_ANGLE()��ʂ��Ė߂��ꂽ�l������ȍ~�����p����K�v������܂�.
	}
	
	// ��]��̍��W���擾����.
	FXY radPos[4];	// �z�񐔂ɒ���.
	GetRotPos(radPos, center, pPos, COUNTOF(radPos), fAngle);

	POINT drawPoint[4];		// �p�x���l�������`�掞�̍��W
	for (int i = 0; i < 4; i++) {
		drawPoint[i] = { static_cast<LONG>(radPos[i].x), static_cast<LONG>(radPos[i].y) };
	}

	// �l�p�`�̕`��.
	RenderRectangle(hdc, drawPoint, rgb);
}


// �~�̕`��.
// ���W(x, y)�ɔ��ar�̉~��`�悷��.
void RenderCircle(HDC hdc, float x, float y, float r, UINT rgb)
{
	int nX = static_cast<int>(x);
	int nY = static_cast<int>(y);
	int nR = static_cast<int>(r);

	// �`��
	HBRUSH hBrush = CreateSolidBrush(rgb);
	HGDIOBJ hOldObj = SelectObject(hdc, hBrush);	// �h��Ԃ��F�ݒ�
	Ellipse(hdc, nX - nR, nY - nR, nX + nR, nY + nR);
	SelectObject(hdc, hOldObj);	// ���̐F�ɕ��A
	DeleteObject(hBrush);
}
