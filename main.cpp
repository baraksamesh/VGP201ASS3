
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	/*viewer.load_mesh_from_file("D:/school/animation/EngineForAnimationCourse/tutorial/data/sphere.obj");
	//viewer.load_mesh_from_file("D:/school/animation/EngineForAnimationCourse/tutorial/data/cube.obj");
	viewer.load_mesh_from_file("D:/school/animation/EngineForAnimationCourse/tutorial/data/ycylinder.obj");
	viewer.load_mesh_from_file("D:/school/animation/EngineForAnimationCourse/tutorial/data/ycylinder.obj");
	viewer.data_list[1].SetCenterOfRotation(Eigen::Vector3f(0, -1, 0));
	viewer.data_list[0].MyTranslate(Eigen::Vector3f(5, 0, -9));
	viewer.data_list[1].MyTranslate(Eigen::Vector3f(0, 0, -9));
	viewer.data_list[2].MyTranslate(Eigen::Vector3f(0, 2*viewer.data_list[1].V.colwise().maxCoeff()[1], -9));
	*/

	viewer.load(4);
	Init(*disp);
	//viewer.initData();
	renderer.init(&viewer);	
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);


	delete disp;
}
