#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
  if (node != NULL) {
    std::cout << "occupancy probability at ";
    std::cout << query << ":\t ";
    std::cout << node->getOccupancy() << std::endl;
  } else {
    std::cout << "occupancy probability at ";
    std::cout << query << ":\t is unknown" << std::endl;
  }
}

int raytrace() {
  // Create empty tree with resolution 0.1
  octomap::OcTree tree (0.1);

  // Insert some measurements of occupied cells
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        const float k = 0.05f;
        octomap::point3d endpoint((float) x * k, (float) y*k, (float) z*k);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Insert some measurements of free cells
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        const float k = 0.02f - 1.0f;
        octomap::point3d endpoint((float) x * k, (float) y * k, (float) z * k);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  // Query
  octomap::point3d query (0., 0., 0.);
  octomap::OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  return 0;
}
