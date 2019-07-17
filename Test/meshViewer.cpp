#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;
PCLVisualizer::Ptr viewer;

void keyboardEventOccurred(const KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym() == "y" && event.keyDown()) // Flat shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_FLAT, "polygon");
  }
  else if (event.getKeySym() == "t" && event.keyDown()) // Gouraud shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_GOURAUD, "polygon");
  }
  else if (event.getKeySym() == "n" && event.keyDown()) // Phong shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_PHONG, "polygon");
  }
}

int main(int argc, char * argv[])
{
  std::vector<int> stl_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".stl");
  if (stl_file_indices.empty())
  {
    PCL_ERROR ("Please provide one STL file as argument\n");
    return 1;
  }

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(argv[stl_file_indices[0]], mesh);

  viewer.reset(new PCLVisualizer);
  viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
  viewer->addPolygonMesh(mesh, "polygon");
  viewer->spin();
  return 0;
}