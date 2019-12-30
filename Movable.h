#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt, Movable* scene);
	void MyRotate(Eigen::Vector3f rotAxis, float angle);
	void MyRotate(Eigen::Vector3f rotAxis, float angle, bool self);
	void MyRotate(Eigen::Vector3f rotAxis, float angle, Movable* scene);
	void MyRotate(Eigen::Matrix3f& mat);
	void MyScale(Eigen::Vector3f amt);
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	Eigen::Matrix4f MakeParentTrans(Movable* parent);
	Eigen::Matrix3f GetRotation();
private:
	Eigen::Transform<float, 3, Eigen::Affine> Tin;
	Eigen::Transform<float, 3, Eigen::Affine> Tout;

};

