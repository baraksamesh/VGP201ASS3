#include "Movable.h"

Movable::Movable()
{
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return (Tout * Tin).matrix();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
}

void Movable::MyTranslate(Eigen::Vector3f amt, Movable* scene)
{
	Eigen::Matrix3f mat = scene->Tout.rotation().matrix();
	mat.transposeInPlace();
	MyTranslate(mat * amt);
}

//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
}


void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle, bool self)
{
	Eigen::Matrix3f mat = Tout.rotation().matrix();
	mat.transposeInPlace();
	if (self)
		Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
	else
		Tout.rotate(Eigen::AngleAxisf(angle, mat * rotAxis));
}

void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle, Movable* scene)
{
	Eigen::Matrix3f mat = scene->Tout.rotation().matrix();
	mat.transposeInPlace();
	MyRotate(mat * rotAxis, angle);
}

void Movable::MyRotate(Eigen::Matrix3f &mat)
{
	Tout.rotate(mat);
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	Tin.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt) {
	Tin.translate(-amt);
	Tout.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation() {
	return -Tin.translation();
}

Eigen::Matrix3f Movable::GetRotation() {
	return Tout.rotation().matrix();

}
